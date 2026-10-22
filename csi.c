/*
 * 仅支持：init() -> capture() -> deinit()
 * Runtime sensor mode selection through init(w=..., h=...).
 */
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/binary.h"
#include "config.h"                 // 用户硬件配置
#include "unity.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_ldo_regulator.h"
#include "driver/isp.h"
#include "example_sensor_init.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_i2c.h"
#include "py/objtuple.h"
#if MICROPY_CAMERA_OV9281
#include "ov9281.h"
#endif
#include "monochrome.h"
#include "esp_private/esp_cache_private.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
/* ---------------- 内部状态 ---------------- */
#undef NUM_CAM_BUFFERS
#define NUM_CAM_BUFFERS   1

static void *cam_buffers[NUM_CAM_BUFFERS] = {NULL};
static size_t cam_buffer_size = 0;

static esp_cam_ctlr_handle_t cam_handle = NULL;
static isp_proc_handle_t     isp_proc   = NULL;
static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static example_sensor_handle_t sensor_handle = {0};
static esp_cam_sensor_device_t *sensor_device;
static const esp_cam_sensor_format_t *active_format;
static bool csi_enabled, csi_started, isp_enabled;
static uint16_t frame_width, frame_height;
static bool h264_mode;
static bool monochrome;
static uint8_t *mono_encoder_input;
static size_t encoder_input_size;
static esp_h264_enc_handle_t h264_handle;
static uint8_t *h264_output;
static uint32_t h264_output_size;

static void h264_check(esp_h264_err_t err) {
    if (err != ESP_H264_ERR_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("H264 encoder error: %d"), err);
    }
}

static mp_obj_t camera_deinit(void);

// Every caller holds the application camera lock, including init/deinit/formats.
static void camera_check(esp_err_t result)
{
    if (result != ESP_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("Camera driver error: %s"), esp_err_to_name(result));
    }
}

static void camera_sensor_open(const char *sensor)
{
    i2c_master_bus_config_t bus = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CSI_MIPI_CSI_CAM_SCCB_SDA_IO,
        .scl_io_num = CSI_MIPI_CSI_CAM_SCCB_SCL_IO,
        .i2c_port = I2C_NUM_0,
        .flags.enable_internal_pullup = true,
    };
    camera_check(i2c_new_master_bus(&bus, &sensor_handle.i2c_bus_handle));
#if MICROPY_CAMERA_OV9281
    if (strcmp(sensor, "OV9281") == 0) {
        // SCCB addresses are 7-bit: SID-low, programmable, SID-high.
        const uint8_t addresses[] = {0x60, 0x70, 0x10};
        for (size_t i = 0; i < MP_ARRAY_SIZE(addresses); ++i) {
            sccb_i2c_config_t io = {
                .scl_speed_hz = 100000, .device_address = addresses[i],
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            };
            camera_check(sccb_new_i2c_io(sensor_handle.i2c_bus_handle, &io,
                                       &sensor_handle.sccb_handle));
            esp_cam_sensor_config_t cfg = {
                .sccb_handle = sensor_handle.sccb_handle,
                .reset_pin = -1, .pwdn_pin = -1, .xclk_pin = -1,
                .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
            };
            sensor_device = ov9281_detect(&cfg);
            if (sensor_device) return;
            camera_check(esp_sccb_del_i2c_io(sensor_handle.sccb_handle));
            sensor_handle.sccb_handle = NULL;
        }
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("OV9281 sensor not detected"));
    }
#endif
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        if (p->port != ESP_CAM_SENSOR_MIPI_CSI) {
            continue;
        }
        sccb_i2c_config_t io = {
            .scl_speed_hz = 100000,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        camera_check(sccb_new_i2c_io(sensor_handle.i2c_bus_handle, &io,
                                   &sensor_handle.sccb_handle));
        esp_cam_sensor_config_t cfg = {
            .sccb_handle = sensor_handle.sccb_handle,
            .reset_pin = -1, .pwdn_pin = -1, .xclk_pin = -1,
            .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
        };
        sensor_device = p->detect(&cfg);
        if (sensor_device) {
            if (strcmp(sensor_device->name, sensor) == 0) return;
            esp_cam_sensor_del_dev(sensor_device);
            sensor_device = NULL;
        }
        camera_check(esp_sccb_del_i2c_io(sensor_handle.sccb_handle));
        sensor_handle.sccb_handle = NULL;
    }
    mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera sensor not detected"));
}

static bool camera_supported_format(const esp_cam_sensor_format_t *fmt)
{
    if (monochrome) {
        // This backport exposes only monochrome RAW8 modes. Never demosaic them.
        return fmt->port == ESP_CAM_SENSOR_MIPI_CSI &&
               fmt->format == ESP_CAM_SENSOR_PIXFORMAT_RAW8;
    }
    return fmt->port == ESP_CAM_SENSOR_MIPI_CSI &&
        (fmt->format == ESP_CAM_SENSOR_PIXFORMAT_RAW8 ||
         fmt->format == ESP_CAM_SENSOR_PIXFORMAT_RAW10) &&
        fmt->isp_info && fmt->isp_info->isp_v1_info.version == 1 &&
        fmt->isp_info->isp_v1_info.bayer_type == ESP_CAM_SENSOR_BAYER_GBRG;
}
// Match full-width Python integers before narrowing to peripheral dimensions.
static const esp_cam_sensor_format_t *camera_find_format(
    const esp_cam_sensor_format_array_t *formats, mp_int_t w, mp_int_t h)
{
    if (w <= 0 || h <= 0 || w > UINT16_MAX || h > UINT16_MAX) {
        return NULL;
    }
    for (int i = 0; i < formats->count; ++i) {
        const esp_cam_sensor_format_t *fmt = &formats->format_array[i];
        if (fmt->width == w && fmt->height == h && camera_supported_format(fmt)) {
            return fmt;
        }
    }
    return NULL;
}

jpeg_encoder_handle_t jpeg_handle;
jpeg_encode_memory_alloc_cfg_t tx_mem_cfg, rx_mem_cfg;
static SemaphoreHandle_t frame_done = NULL;
uint32_t raw_size_1080p;
uint32_t jpg_size_1080p;
/* ---------------- 回调 ---------------- */
static bool camera_trans_finished(esp_cam_ctlr_handle_t handle,
                                  esp_cam_ctlr_trans_t *trans,
                                  void *user_data)
{
    BaseType_t task_woken = pdFALSE;
    if (trans->buffer == cam_buffers[0] &&
        trans->received_size == cam_buffer_size) {
        xSemaphoreGiveFromISR(frame_done, &task_woken);
    }
    return task_woken == pdTRUE;
}

/* -------------- init -------------- */
static mp_obj_t camera_init_inner(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_w, ARG_h, ARG_h264, ARG_fps, ARG_gop, ARG_bitrate, ARG_sensor };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_w, MP_ARG_INT, {.u_int = CSI_MIPI_CSI_DISP_HRES} },
        { MP_QSTR_h, MP_ARG_INT, {.u_int = CSI_MIPI_CSI_DISP_VRES} },
        { MP_QSTR_h264, MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_fps, MP_ARG_INT, {.u_int = 20} },
        { MP_QSTR_gop, MP_ARG_INT, {.u_int = 10} },
        { MP_QSTR_bitrate, MP_ARG_INT, {.u_int = 2000000} },
        { MP_QSTR_sensor, MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_QSTR(MP_QSTR_OV5647)} },
    };
        /* H264 settings are target rates; the application paces capture. */
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const char *sensor = mp_obj_str_get_str(args[ARG_sensor].u_obj);
    monochrome = strcmp(sensor, "OV9281") == 0;
    if (strcmp(sensor, "OV5647") != 0 && !monochrome) {
        mp_raise_ValueError(MP_ERROR_TEXT("sensor must be OV5647 or OV9281"));
    }
#if !MICROPY_CAMERA_OV9281
    if (monochrome) {
        mp_raise_ValueError(MP_ERROR_TEXT("OV9281 driver not built"));
    }
#endif
    const mp_int_t w = args[ARG_w].u_int;
    const mp_int_t h = args[ARG_h].u_int;

    if (w <= 0 || h <= 0 || w > UINT16_MAX || h > UINT16_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("Unsupported camera dimensions"));
    }
    h264_mode = args[ARG_h264].u_bool;
    if (h264_mode && ((w % 16) || (h % 16) || w > 1920 || h > 2032 ||
        args[ARG_fps].u_int < 1 || args[ARG_fps].u_int > 255 ||
        args[ARG_gop].u_int < 1 || args[ARG_gop].u_int > 255 ||
        args[ARG_bitrate].u_int < 10000 || args[ARG_bitrate].u_int > 20000000)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid H264 dimensions, fps, gop or bitrate"));
    }
    frame_done = xSemaphoreCreateBinary();
    if (frame_done == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Frame semaphore alloc failed"));
    }


    /* 1. LDO */
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id    = CSI_USED_LDO_CHAN_ID,
        .voltage_mv = CSI_USED_LDO_VOLTAGE_MV,
    };
    if (esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("LDO acquire failed"));
    }
    mp_printf(&mp_plat_print, "LDO initialized\n");
    camera_sensor_open(sensor);
    esp_cam_sensor_format_array_t formats = {0};
    camera_check(esp_cam_sensor_query_format(sensor_device, &formats));
    active_format = camera_find_format(&formats, w, h);
    if (!active_format) {
        mp_raise_ValueError(MP_ERROR_TEXT("Unsupported camera dimensions"));
    }
    camera_check(esp_cam_sensor_set_format(sensor_device, active_format));
    if (h264_mode && args[ARG_fps].u_int > active_format->fps) {
        mp_raise_ValueError(MP_ERROR_TEXT("H264 fps exceeds sensor rate"));
    }
    frame_width = w;
    frame_height = h;
    encoder_input_size = h264_mode ? (size_t)w * h * 3 / 2 : (size_t)w * h * 2;
    cam_buffer_size = monochrome ? (size_t)w * h : encoder_input_size;
    /* 2. 帧缓冲 */
    size_t align = 0;
    // Cache invalidation must use the external-memory cache-line alignment.
    if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA, &align) != ESP_OK || !align) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera cache alignment unavailable"));
    }
    for (int i = 0; i < NUM_CAM_BUFFERS; ++i) {
        cam_buffers[i] = heap_caps_aligned_calloc(align, 1, cam_buffer_size,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (!cam_buffers[i]) {
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Buffer alloc failed"));
        }
    	mp_printf(&mp_plat_print, "Camera buffer %d allocated: %p\n", i, cam_buffers[i]);
    }
    mp_printf(&mp_plat_print, "%d camera buffers allocated\n", NUM_CAM_BUFFERS);
    if (monochrome) {
        mono_encoder_input = heap_caps_aligned_calloc(align, 1, encoder_input_size,
                                                      MALLOC_CAP_SPIRAM);
        if (!mono_encoder_input) {
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Monochrome buffer alloc failed"));
        }
    }
    /* 4. CSI 控制器 */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = w,
        .v_res                  = h,
        .lane_bit_rate_mbps     = active_format->mipi_info.mipi_clk / 1000000,
        .input_data_color_type  = active_format->format == ESP_CAM_SENSOR_PIXFORMAT_RAW10
                                  ? CAM_CTLR_COLOR_RAW10 : CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = monochrome ? CAM_CTLR_COLOR_RAW8 :
                                  (h264_mode ? CAM_CTLR_COLOR_YUV420 : CAM_CTLR_COLOR_RGB565),
        .data_lane_num          = active_format->mipi_info.lane_num,
        .byte_swap_en           = false,
        .queue_items            = 1,
    };
    if (esp_cam_new_csi_ctlr(&csi_cfg, &cam_handle) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("CSI init failed"));
    }
    mp_printf(&mp_plat_print, "CSI controller initialized\n");
   

    esp_cam_ctlr_evt_cbs_t cbs = {
        // With no queued user buffer, CSI writes into its internal backup.
        // A user buffer is submitted only by capture(), exactly once.
        .on_get_new_trans  = NULL,
        .on_trans_finished = camera_trans_finished,
    };
    camera_check(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL));
    camera_check(esp_cam_ctlr_enable(cam_handle));
    csi_enabled = true;
    mp_printf(&mp_plat_print, "Camera event callbacks registered\n");
    /* 5. ISP */
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = active_format->format == ESP_CAM_SENSOR_PIXFORMAT_RAW10
                                  ? ISP_COLOR_RAW10 : ISP_COLOR_RAW8,
        .bayer_order            = COLOR_RAW_ELEMENT_ORDER_GBRG,
        .output_data_color_type = monochrome ? ISP_COLOR_RAW8 :
                                  (h264_mode ? ISP_COLOR_YUV420 : ISP_COLOR_RGB565),
        .flags.bypass_isp = monochrome,
        .has_line_start_packet  = active_format->mipi_info.line_sync_en,
        .has_line_end_packet    = active_format->mipi_info.line_sync_en,
        .h_res                  = w,
        .v_res                  = h,
    };
    camera_check(esp_isp_new_processor(&isp_cfg, &isp_proc));
    // Bypass is configured by new_processor; IDF forbids enabling it.
    if (!monochrome) {
        camera_check(esp_isp_enable(isp_proc));
        isp_enabled = true;
    }
    mp_printf(&mp_plat_print, "ISP processor initialized\n");
    /* 6. 启动 */
    if (esp_cam_ctlr_start(cam_handle) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera start failed"));
    }
    csi_started = true;
    int stream_on = 1;
    camera_check(esp_cam_sensor_ioctl(sensor_device, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on));
    mp_printf(&mp_plat_print, "Camera capture started\n");
    if (h264_mode) {
        esp_h264_enc_cfg_hw_t cfg = {
            .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
            .gop = args[ARG_gop].u_int, .fps = args[ARG_fps].u_int,
            .res = {.width = w, .height = h},
            .rc = {.bitrate = args[ARG_bitrate].u_int, .qp_min = 20, .qp_max = 40},
        };
        h264_check(esp_h264_enc_hw_new(&cfg, &h264_handle));
        h264_check(esp_h264_enc_open(h264_handle));
        h264_output = esp_h264_aligned_calloc(64, 1, encoder_input_size,
                                            &h264_output_size, ESP_H264_MEM_SPIRAM);
        if (!h264_output) {
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("H264 output allocation failed"));
        }
        return mp_const_none;
    }
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 1000,
    };

    rx_mem_cfg.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;

    tx_mem_cfg.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER;

    if (jpeg_new_encoder_engine(&encode_eng_cfg, &jpeg_handle) != ESP_OK){
    	mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Jpeg encoder init failed"));
    }
    mp_printf(&mp_plat_print, "Jpeg encoder initialized\n");
    return mp_const_none;
}
// Keep the exception guard outside argument parsing so all partial resources
// are released, without tearing down an already-running camera on misuse.
static mp_obj_t camera_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    if (cam_handle || sensor_device) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera already initialized"));
    }
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t result = camera_init_inner(n_args, pos_args, kw_args);
        nlr_pop();
        return result;
    }
    camera_deinit();
    nlr_jump(nlr.ret_val);
}
MP_DEFINE_CONST_FUN_OBJ_KW(camera_init_obj, 0, camera_init);

static mp_obj_t camera_formats(void)
{
    if (!sensor_device) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Initialize camera before querying formats"));
    }
    esp_cam_sensor_format_array_t formats = {0};
    camera_check(esp_cam_sensor_query_format(sensor_device, &formats));
    mp_obj_t result = mp_obj_new_list(0, NULL);
    for (int i = 0; i < formats.count; ++i) {
        const esp_cam_sensor_format_t *fmt = &formats.format_array[i];
        if (!camera_supported_format(fmt)) {
            continue;
        }
        mp_obj_t entry[] = {
            mp_obj_new_int(fmt->width), mp_obj_new_int(fmt->height),
            mp_obj_new_int(fmt->fps),
            mp_obj_new_str(fmt->format == ESP_CAM_SENSOR_PIXFORMAT_RAW10 ? "RAW10" : "RAW8",
                           fmt->format == ESP_CAM_SENSOR_PIXFORMAT_RAW10 ? 5 : 4),
        };
        mp_obj_list_append(result, mp_obj_new_tuple(MP_ARRAY_SIZE(entry), entry));
    }
    return result;
}
MP_DEFINE_CONST_FUN_OBJ_0(camera_formats_obj, camera_formats);

/* -------------- deinit -------------- */
static mp_obj_t camera_deinit(void)
{
    if (sensor_device) {
        int stream_off = 0;
        esp_cam_sensor_ioctl(sensor_device, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_off);
    }
    if (cam_handle) {
        if (csi_started) esp_cam_ctlr_stop(cam_handle);
        if (csi_enabled) esp_cam_ctlr_disable(cam_handle);
        esp_cam_ctlr_del(cam_handle);
        cam_handle = NULL;
    }
    if (isp_proc) {
        if (isp_enabled) esp_isp_disable(isp_proc);
        esp_isp_del_processor(isp_proc);
        isp_proc = NULL;
    }
    if (h264_handle) {
        esp_h264_enc_close(h264_handle);
        esp_h264_enc_del(h264_handle);
        h264_handle = NULL;
    }
    if (h264_output) {
        esp_h264_free(h264_output);
        h264_output = NULL;
    }
    h264_output_size = 0;
    h264_mode = false;
    monochrome = false;
    free(mono_encoder_input);
    mono_encoder_input = NULL;
    encoder_input_size = 0;
    if (jpeg_handle) {
        jpeg_del_encoder_engine(jpeg_handle);
        jpeg_handle = NULL;
    }
    if (frame_done) {
        vSemaphoreDelete(frame_done);
        frame_done = NULL;
    }
    if (sensor_device) {
        esp_cam_sensor_del_dev(sensor_device);
        sensor_device = NULL;
    }
    if (sensor_handle.sccb_handle) {
        esp_sccb_del_i2c_io(sensor_handle.sccb_handle);
        sensor_handle.sccb_handle = NULL;
    }
    if (sensor_handle.i2c_bus_handle) {
        i2c_del_master_bus(sensor_handle.i2c_bus_handle);
        sensor_handle.i2c_bus_handle = NULL;
    }
    active_format = NULL;
    csi_enabled = csi_started = isp_enabled = false;
    frame_width = frame_height = 0;
    if (ldo_mipi_phy) {
        esp_ldo_release_channel(ldo_mipi_phy);
        ldo_mipi_phy = NULL;
    }
    for (int i = 0; i < NUM_CAM_BUFFERS; ++i) {
        if (cam_buffers[i]) {
            heap_caps_free(cam_buffers[i]);
            cam_buffers[i] = NULL;
        }
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(camera_deinit_obj, camera_deinit);

// Called after Python threads stop, before MicroPython tears down its VM.
void camera_deinit_all(void)
{
    camera_deinit();
}

static mp_obj_t camera_capture(void)
{
    if (!cam_buffers[0] || !frame_done) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera not initialized"));
    }

    // Discard any stale completion, then queue one complete frame.
    // ESP-IDF switches DMA back to its internal backup before notifying us.
    xSemaphoreTake(frame_done, 0);
    esp_cam_ctlr_trans_t transaction = {
        .buffer = cam_buffers[0],
        .buflen = cam_buffer_size,
    };
    if (esp_cam_ctlr_receive(cam_handle, &transaction, 1000) != ESP_OK ||
        xSemaphoreTake(frame_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
        // Stop DMA before freeing a buffer that could still be queued/in use.
        camera_deinit();
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera frame timeout; restart board"));
    }
    // CSI DMA writes PSRAM without updating the CPU cache. In the vendor's
    // ESP-IDF revision the completion callback can invalidate zero bytes.
    // The buffer is now owned by capture(), so invalidate the full image.
    if (esp_cache_msync(cam_buffers[0], cam_buffer_size,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera cache invalidate failed"));
    }

    uint8_t *encoder_input = cam_buffers[0];
    if (monochrome) {
        camera_mono_convert(cam_buffers[0], mono_encoder_input,
                            (size_t)frame_width * frame_height, h264_mode);
        encoder_input = mono_encoder_input;
        if (h264_mode) {
            camera_check(esp_cache_msync(encoder_input, encoder_input_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M));
        }
    }
    if (h264_mode) {
        esp_h264_enc_in_frame_t input = {
            .raw_data = {.buffer = encoder_input, .len = encoder_input_size},
        };
        esp_h264_enc_out_frame_t output = {
            .raw_data = {.buffer = h264_output, .len = h264_output_size},
        };
        esp_h264_err_t err = esp_h264_enc_process(h264_handle, &input, &output);
        if (err != ESP_H264_ERR_OK) {
            // Fail closed: never continue sending a broken prediction chain.
            camera_deinit();
            h264_check(err);
        }
        mp_obj_t result[] = {
            mp_obj_new_bytes(h264_output, output.length),
            mp_obj_new_bool(output.frame_type == ESP_H264_FRAME_TYPE_IDR),
        };
        return mp_obj_new_tuple(2, result);
    }
    // 获取原始图像数据大小
    raw_size_1080p = encoder_input_size;

    // 分配内存用于存储原始图像数据
    size_t tx_buffer_size = 0;
    uint8_t *raw_buf_1080p = (uint8_t*)jpeg_alloc_encoder_mem(raw_size_1080p, &tx_mem_cfg, &tx_buffer_size);
    if (raw_buf_1080p == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Failed to allocate memory for raw buffer"));    
    }

    // 将相机缓冲区的内容复制到分配的内存中
    memcpy(raw_buf_1080p, encoder_input, raw_size_1080p);

    // 分配内存用于存储 JPEG 数据
    size_t rx_buffer_size = 0;
    uint8_t *jpg_buf_1080p = (uint8_t*)jpeg_alloc_encoder_mem(raw_size_1080p, &rx_mem_cfg, &rx_buffer_size);
    if (jpg_buf_1080p == NULL) {
        free(raw_buf_1080p);
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Failed to allocate memory for JPEG buffer"));
        
    }

    // 配置 JPEG 编码器
    jpeg_encode_cfg_t enc_config = {
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = 80,
        .width = frame_width,
        .height = frame_height,
    };

    if (jpeg_encoder_process(jpeg_handle, &enc_config, raw_buf_1080p, raw_size_1080p, jpg_buf_1080p, rx_buffer_size, &jpg_size_1080p) != ESP_OK){
        free(raw_buf_1080p);
        free(jpg_buf_1080p);
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Failed"));
    }

    // 返回 JPEG 数据
    mp_obj_t jpg_data = mp_obj_new_bytes(jpg_buf_1080p, jpg_size_1080p);

    // 释放分配的内存
    free(raw_buf_1080p);
    free(jpg_buf_1080p);
    //free(enc_config);
    return jpg_data;
}
MP_DEFINE_CONST_FUN_OBJ_0(camera_capture_obj, camera_capture);

/* -------------- 模块表 -------------- */
static const mp_rom_map_elem_t camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),    MP_ROM_PTR(&camera_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),  MP_ROM_PTR(&camera_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_formats), MP_ROM_PTR(&camera_formats_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&camera_capture_obj) },
};
static MP_DEFINE_CONST_DICT(camera_globals, camera_globals_table);

const mp_obj_module_t camera_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&camera_globals,
};

MP_REGISTER_MODULE(MP_QSTR_camera, camera_user_cmodule);
