/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"

/* Example configurations */
#define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_VOICE_VOLUME    CONFIG_EXAMPLE_VOICE_VOLUME

/* Shared I2C bus: ES8311 codec + PCA9535 IO expander */
#define I2C_NUM             (1)
#define I2C_SCL_IO          (GPIO_NUM_21)
#define I2C_SDA_IO          (GPIO_NUM_20)
#define PCA9535_I2C_ADDR    (0x20)

/* Power amplifier enable pin (active high) */
#define EXAMPLE_PA_CTRL_IO  (GPIO_NUM_53)

/* I2S port and GPIOs (ES8311 playback path) */
#define I2S_NUM             (0)
#define I2S_MCK_IO          (GPIO_NUM_33)
#define I2S_BCK_IO          (GPIO_NUM_32)
#define I2S_WS_IO           (GPIO_NUM_31)
#define I2S_DO_IO           (GPIO_NUM_30)
