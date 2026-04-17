/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "sdkconfig.h"

/* Example configurations */
#define EXAMPLE_RECV_BUF_SIZE   (2400) 
#define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_MCLK_MULTIPLE   (256) // Changed to 256, as ES7210 does not support 384x frequency at 16kHz
#define EXAMPLE_MCLK_FREQ_HZ    (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)
#define EXAMPLE_VOICE_VOLUME    70 // Lower the default volume from 60 to 30 (Note: The value is currently set to 70)
#if CONFIG_EXAMPLE_MODE_ECHO
#define EXAMPLE_MIC_GAIN        CONFIG_EXAMPLE_MIC_GAIN
#endif

#if !defined(CONFIG_EXAMPLE_BSP)

/* PCA9535 Configuration */
#define MISC_I2C_SDA (GPIO_NUM_20)
#define MISC_I2C_SCL (GPIO_NUM_21)
#define PCA9535_I2C_ADDR 0x20

/* I2C port and GPIOs */
#define I2C_NUM         (0)
#if CONFIG_IDF_TARGET_ESP32P4
#define I2C_SCL_IO      (GPIO_NUM_21)
#define I2C_SDA_IO      (GPIO_NUM_20)
#define GPIO_OUTPUT_PA  (GPIO_NUM_53)
#endif

/* I2S port and GPIOs for ES8311 (Speaker) */
#define I2S_NUM         (0)
#if CONFIG_IDF_TARGET_ESP32P4
#define I2S_MCK_IO      (GPIO_NUM_33)
#define I2S_BCK_IO      (GPIO_NUM_32)
#define I2S_WS_IO       (GPIO_NUM_31)
#define I2S_DO_IO       (GPIO_NUM_30)
#define I2S_DI_IO       (GPIO_NUM_11)
#endif

/* I2S port and GPIOs for ES7210 (Microphone) */
#define ES7210_I2S_NUM    I2S_NUM_1
#define ES7210_I2S_MCK_IO GPIO_NUM_29
#define ES7210_I2S_BCK_IO GPIO_NUM_28
#define ES7210_I2S_WS_IO  GPIO_NUM_27
#define ES7210_I2S_DI_IO  GPIO_NUM_26

/* Record & Play Configuration */
#define RECORD_TIME_SEC 10
/* Buffer Size = SampleRate * 2bytes(16bit) * 2channels(Stereo) * Seconds */
#define RECORD_BUFFER_SIZE (EXAMPLE_SAMPLE_RATE * 2 * 2 * RECORD_TIME_SEC)

#else // CONFIG_EXAMPLE_BSP
#include "bsp/esp-bsp.h"
#define I2C_NUM BSP_I2C_NUM

#endif // CONFIG_EXAMPLE_BSP


