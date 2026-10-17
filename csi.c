/*
 * 仅支持：init() -> capture() -> deinit()
 * 分辨率由关键字参数 w / h 传入（暂未实现），其余在 config.h
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
#include "esp_private/esp_cache_private.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
/* ---------------- 内部状态 ---------------- */
#define NUM_CAM_BUFFERS   2

static void *cam_buffers[NUM_CAM_BUFFERS] = {NULL};
static size_t cam_buffer_size = 0;

static esp_cam_ctlr_handle_t cam_handle = NULL;
static isp_proc_handle_t     isp_proc   = NULL;
static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static example_sensor_handle_t sensor_handle = {0};
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
static mp_obj_t camera_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_w, ARG_h };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_w, MP_ARG_INT, {.u_int = CSI_MIPI_CSI_DISP_HRES} },
        { MP_QSTR_h, MP_ARG_INT, {.u_int = CSI_MIPI_CSI_DISP_VRES} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const uint16_t w = args[ARG_w].u_int;
    const uint16_t h = args[ARG_h].u_int;

    // The sensor format and JPEG dimensions are fixed by config.h.
    if (w != CSI_MIPI_CSI_DISP_HRES || h != CSI_MIPI_CSI_DISP_VRES) {
        mp_raise_ValueError(MP_ERROR_TEXT("Unsupported camera dimensions"));
    }
    if (cam_handle != NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera already initialized"));
    }
    frame_done = xSemaphoreCreateBinary();
    if (frame_done == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Frame semaphore alloc failed"));
    }

    cam_buffer_size = w * h * 2;          // RGB565

    /* 1. LDO */
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id    = CSI_USED_LDO_CHAN_ID,
        .voltage_mv = CSI_USED_LDO_VOLTAGE_MV,
    };
    if (esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("LDO acquire failed"));
    }
    mp_printf(&mp_plat_print, "LDO initialized\n");
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
    /* 3. 传感器 */
    example_sensor_config_t sensor_cfg = {
        .i2c_port_num   = I2C_NUM_0,
        .i2c_sda_io_num = CSI_MIPI_CSI_CAM_SCCB_SDA_IO,
        .i2c_scl_io_num = CSI_MIPI_CSI_CAM_SCCB_SCL_IO,
        .port           = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name    = CSI_CAM_FORMAT,
    };
    example_sensor_init(&sensor_cfg, &sensor_handle);
    mp_printf(&mp_plat_print, "Camera sensor initialized\n");

    /* 4. CSI 控制器 */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = w,
        .v_res                  = h,
        .lane_bit_rate_mbps     = CSI_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num          = 2,
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
    if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL) != ESP_OK ||
        esp_cam_ctlr_enable(cam_handle) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("CSI enable failed"));
    }
    mp_printf(&mp_plat_print, "Camera event callbacks registered\n");
    /* 5. ISP */
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet  = true,
        .has_line_end_packet    = true,
        .h_res                  = w,
        .v_res                  = h,
    };
    if (esp_isp_new_processor(&isp_cfg, &isp_proc) != ESP_OK ||
        esp_isp_enable(isp_proc) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("ISP init failed"));
    }
    mp_printf(&mp_plat_print, "ISP processor initialized\n");
    /* 6. 启动 */
    if (esp_cam_ctlr_start(cam_handle) != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Camera start failed"));
    }
    mp_printf(&mp_plat_print, "Camera capture started\n");
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 70,
    };

    rx_mem_cfg.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;

    tx_mem_cfg.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER;

    if (jpeg_new_encoder_engine(&encode_eng_cfg, &jpeg_handle) != ESP_OK){
    	mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Jpeg encoder init failed"));
    }
    mp_printf(&mp_plat_print, "Jpeg encoder initialized\n");
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(camera_init_obj, 0, camera_init);

/* -------------- deinit -------------- */
static mp_obj_t camera_deinit(void)
{
    if (cam_handle) {
        esp_cam_ctlr_stop(cam_handle);
        esp_cam_ctlr_disable(cam_handle);
        esp_cam_ctlr_del(cam_handle);
        cam_handle = NULL;
    }
    if (isp_proc) {
        esp_isp_disable(isp_proc);
        esp_isp_del_processor(isp_proc);
        isp_proc = NULL;
    }
    if (jpeg_handle) {
        jpeg_del_encoder_engine(jpeg_handle);
        jpeg_handle = NULL;
    }
    if (frame_done) {
        vSemaphoreDelete(frame_done);
        frame_done = NULL;
    }
    if (sensor_handle.i2c_bus_handle && sensor_handle.sccb_handle) {
        example_sensor_deinit(sensor_handle);
        sensor_handle = (example_sensor_handle_t){0};
    }
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

    // 获取原始图像数据大小
    raw_size_1080p = cam_buffer_size;

    // 分配内存用于存储原始图像数据
    size_t tx_buffer_size = 0;
    uint8_t *raw_buf_1080p = (uint8_t*)jpeg_alloc_encoder_mem(raw_size_1080p, &tx_mem_cfg, &tx_buffer_size);
    if (raw_buf_1080p == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Failed to allocate memory for raw buffer"));    
    }

    // 将相机缓冲区的内容复制到分配的内存中
    memcpy(raw_buf_1080p, cam_buffers[0], raw_size_1080p);

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
        .width = CSI_MIPI_CSI_DISP_HRES,
        .height = CSI_MIPI_CSI_DISP_VRES,
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
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&camera_capture_obj) },
};
static MP_DEFINE_CONST_DICT(camera_globals, camera_globals_table);

const mp_obj_module_t camera_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&camera_globals,
};

MP_REGISTER_MODULE(MP_QSTR_camera, camera_user_cmodule);
