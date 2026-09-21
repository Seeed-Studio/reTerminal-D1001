/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief BSP Touchscreen
 *
 * This file offers API for basic touchscreen initialization.
 * It is useful for users who want to use the touchscreen without the default Graphical Library LVGL.
 *
 * For standard LCD initialization with LVGL graphical library, you can call all-in-one function bsp_display_start().
 */

#pragma once
#include "esp_lcd_touch.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP touch configuration structure
 *
 */
typedef struct {
    void *dummy;    /*!< Prepared for future use. */
} bsp_touch_config_t;

/**
 * @brief Create new touchscreen
 *
 * If you want to free resources allocated by this function, you can use esp_lcd_touch API, ie.:
 *
 * \code{.c}
 * esp_lcd_touch_del(tp);
 * \endcode
 *
 * @param[in]  config    touch configuration
 * @param[out] ret_touch esp_lcd_touch touchscreen handle
 * @return
 *      - ESP_OK         On success
 *      - Else           esp_lcd_touch failure
 */
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);

/**
 * @brief Touch INT pin report-rate statistics
 *
 * Collected on the touch controller INT pin (BSP_LCD_TOUCH_INT) without
 * interfering with the LVGL touch read path.
 */
typedef struct {
    uint32_t falling_hz;       /*!< Falling edges in the last 1 s window (touch report rate) */
    uint32_t rising_hz;        /*!< Rising edges in the last 1 s window (waveform diagnostics) */
    uint32_t min_interval_ms;  /*!< Minimum falling-edge interval (0 if fewer than 2 edges) */
    uint32_t max_interval_ms;  /*!< Maximum falling-edge interval (0 if fewer than 2 edges) */
} bsp_touch_int_rate_info_t;

/**
 * @brief Start measuring the touch INT pin report rate (idempotent)
 *
 * Configures the touch INT GPIO as an input with ANYEDGE interrupts and
 * starts a 1 Hz statistics window. Safe to call again after already started.
 *
 * @return
 *      - ESP_OK         On success (or already started)
 *      - Else           GPIO / esp_timer failure
 */
esp_err_t bsp_touch_int_rate_start(void);

/**
 * @brief Get the touch INT report rate of the last completed 1 s window
 *
 * Thread-safe, can be called from any task context.
 *
 * @param[out] info Statistics of the last completed window (all zeros until
 *                  the first window completes)
 */
void bsp_touch_int_rate_get_info(bsp_touch_int_rate_info_t *info);

#ifdef __cplusplus
}
#endif
