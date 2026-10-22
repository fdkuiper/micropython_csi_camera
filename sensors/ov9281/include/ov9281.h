/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor_types.h"
#include "ov9281_types.h"

// 7-bit SCCB addresses are probed explicitly in csi.c.
#define OV9281_PID         0x9281
#define OV9281_SENSOR_NAME "OV9281"

/**
 * @brief Power on camera sensor device and detect the device connected to the designated sccb bus.
 *
 * @param[in] config Configuration related to device power-on and detection.
 * @return
 *      - Camera device handle on success, otherwise, failed.
 */
esp_cam_sensor_device_t *ov9281_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
