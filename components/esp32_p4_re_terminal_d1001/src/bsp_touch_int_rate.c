/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileIdentifier: Apache-2.0
 */

/**
 * @file bsp_touch_int_rate.c
 * @brief Touch INT pin (GPIO16, GSL3670 interrupt output) report-rate measurement.
 *
 * This module is a pure observer: it counts edges on the touch controller INT
 * pin and provides the falling-edge frequency of the last completed 1 s
 * window. It does NOT take part in the LVGL touch read path, so it can be
 * enabled without affecting normal touch behavior.
 */

#include <stdint.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "esp32_p4_re_terminal_d1001.h"
#include "touch.h"

static const char *TAG = "touch_int";

/* Statistics window length */
#define TOUCH_INT_RATE_WINDOW_US    (1000 * 1000)

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_started = false;

/* Current (accumulating) window, written from the ISR only */
static volatile bool s_line_high = true;          /* expected line level (edges alternate) */
static volatile uint32_t s_falling_cnt = 0;
static volatile uint32_t s_rising_cnt = 0;
static volatile uint32_t s_min_interval_ms = UINT32_MAX;
static volatile uint32_t s_max_interval_ms = 0;
static volatile int64_t s_last_fall_us = -1;

/* Snapshot of the last completed window, read by applications */
static bsp_touch_int_rate_info_t s_info = {0};

static esp_timer_handle_t s_window_timer = NULL;

static void touch_int_isr(void *arg);
static void touch_int_window_cb(void *arg);

esp_err_t bsp_touch_int_rate_start(void)
{
    esp_err_t ret;

    if (s_started) {
        return ESP_OK;
    }

    portENTER_CRITICAL(&s_lock);
    s_falling_cnt = 0;
    s_rising_cnt = 0;
    s_min_interval_ms = UINT32_MAX;
    s_max_interval_ms = 0;
    s_last_fall_us = -1;
    s_info.falling_hz = 0;
    s_info.rising_hz = 0;
    s_info.min_interval_ms = 0;
    s_info.max_interval_ms = 0;
    portEXIT_CRITICAL(&s_lock);

    /* Get in sync with the current line state before enabling the interrupt,
     * so the edge classifier starts aligned with the pin level. */
    s_line_high = (gpio_get_level(BSP_LCD_TOUCH_INT) != 0);

    /* INT pin: input with internal pull-up, interrupt on both edges so both
     * the falling (report) and rising (pulse end) edges can be counted. */
    const gpio_config_t int_gpio_config = {
        .pin_bit_mask = 1ULL << BSP_LCD_TOUCH_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_gpio_config), TAG, "INT GPIO config failed");

    /* The GPIO ISR service may already be installed by other BSP code */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Install GPIO ISR service failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BSP_LCD_TOUCH_INT, touch_int_isr, NULL),
                        TAG, "Add touch INT ISR failed");

    /* Move the ISR counters into a stable snapshot once per second */
    const esp_timer_create_args_t timer_args = {
        .callback = touch_int_window_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "touch_int_rate",
        .skip_unhandled_events = true,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_window_timer), TAG, "Create window timer failed");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_window_timer, TOUCH_INT_RATE_WINDOW_US),
                        TAG, "Start window timer failed");

    s_started = true;
    ESP_LOGI(TAG, "Touch INT rate measurement started on GPIO%d", BSP_LCD_TOUCH_INT);

    return ESP_OK;
}

void bsp_touch_int_rate_get_info(bsp_touch_int_rate_info_t *info)
{
    if (info == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    *info = s_info;
    portEXIT_CRITICAL(&s_lock);
}

static void IRAM_ATTR touch_int_isr(void *arg)
{
    (void)arg;

    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&s_lock);

    /* Edges on a digital input strictly alternate, so the current level can
     * be derived without calling flash-resident driver code from the ISR. */
    if (s_line_high) {
        /* Falling edge: one touch report from the GSL3670 */
        s_line_high = false;
        s_falling_cnt++;
        if (s_last_fall_us >= 0) {
            const uint32_t interval_ms = (uint32_t)((now_us - s_last_fall_us) / 1000);
            if (interval_ms < s_min_interval_ms) {
                s_min_interval_ms = interval_ms;
            }
            if (interval_ms > s_max_interval_ms) {
                s_max_interval_ms = interval_ms;
            }
        }
        s_last_fall_us = now_us;
    } else {
        /* Rising edge: end of the INT pulse */
        s_line_high = true;
        s_rising_cnt++;
    }

    portEXIT_CRITICAL_ISR(&s_lock);
}

static void touch_int_window_cb(void *arg)
{
    (void)arg;

    portENTER_CRITICAL(&s_lock);
    s_info.falling_hz = s_falling_cnt;
    s_info.rising_hz = s_rising_cnt;
    s_info.min_interval_ms = (s_min_interval_ms == UINT32_MAX) ? 0 : s_min_interval_ms;
    s_info.max_interval_ms = s_max_interval_ms;
    /* Reset for the next window */
    s_falling_cnt = 0;
    s_rising_cnt = 0;
    s_min_interval_ms = UINT32_MAX;
    s_max_interval_ms = 0;
    portEXIT_CRITICAL(&s_lock);
}
