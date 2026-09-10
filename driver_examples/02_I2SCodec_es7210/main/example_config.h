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
#define EXAMPLE_VOICE_VOLUME    (70)
#define EXAMPLE_MIC_GAIN_DB     (30.0f)

/* Shared I2C bus: ES8311/ES7210 codecs + PCA9535 IO expander */
#define I2C_NUM             (1)
#define I2C_SCL_IO          (GPIO_NUM_21)
#define I2C_SDA_IO          (GPIO_NUM_20)
#define PCA9535_I2C_ADDR    (0x20)

/* Power amplifier enable pin (active high) */
#define EXAMPLE_PA_CTRL_IO  (GPIO_NUM_53)

/* I2S port and GPIOs for ES8311 (Speaker) */
#define ES8311_I2S_NUM      (0)
#define ES8311_I2S_MCK_IO   (GPIO_NUM_33)
#define ES8311_I2S_BCK_IO   (GPIO_NUM_32)
#define ES8311_I2S_WS_IO    (GPIO_NUM_31)
#define ES8311_I2S_DO_IO    (GPIO_NUM_30)

/* I2S port and GPIOs for ES7210 (Microphone), TDM 4-channel mode
 * (same as the factory firmware) */
#define ES7210_I2S_NUM      (1)
#define ES7210_I2S_MCK_IO   (GPIO_NUM_29)
#define ES7210_I2S_BCK_IO   (GPIO_NUM_28)
#define ES7210_I2S_WS_IO    (GPIO_NUM_27)
#define ES7210_I2S_DI_IO    (GPIO_NUM_26)
#define ES7210_TDM_CHANNELS (4)

/* Record & Play Configuration */
#define RECORD_TIME_SEC     (10)
/* Record buffer = SampleRate * 2bytes(16bit) * 4channels(TDM) * Seconds */
#define RECORD_BUFFER_SIZE  (EXAMPLE_SAMPLE_RATE * 2 * ES7210_TDM_CHANNELS * RECORD_TIME_SEC)
/* Play buffer = SampleRate * 2bytes(16bit) * 2channels(Stereo) * Seconds */
#define PLAY_BUFFER_SIZE    (EXAMPLE_SAMPLE_RATE * 2 * 2 * RECORD_TIME_SEC)
