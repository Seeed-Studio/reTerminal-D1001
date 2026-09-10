
// ==================== Record and Play Loop Test (ES8311 + ES7210) ====================

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"
#include "example_config.h"

static const char *TAG = "record_play_es7210_es8311";

/* I2S channel handles */
static i2s_chan_handle_t tx_handle = NULL;   // ES8311 playback (speaker)
static i2s_chan_handle_t rx_handle = NULL;   // ES7210 record (microphone)
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static esp_codec_dev_handle_t play_dev = NULL;
static esp_codec_dev_handle_t record_dev = NULL;

static esp_err_t pca9535_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data)
{
    const uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(dev, write_buf, sizeof(write_buf), 1000);
}

/* 1. Power up the audio path through the PCA9535 IO expander:
 * P10 (PWR_HOLD) keeps the 3.3V rail on, P13 (POWER_AMP_EN) powers the amplifier */
static esp_err_t board_power_init(void)
{
    /* The ES8311/ES7210 codecs and the PCA9535 IO expander share the same I2C bus */
    const i2c_master_bus_config_t i2c_mst_cfg = {
        .i2c_port = I2C_NUM,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_mst_cfg, &i2c_bus_handle), TAG, "create i2c bus failed");

    i2c_master_dev_handle_t pca9535_dev = NULL;
    const i2c_device_config_t pca9535_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9535_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_handle, &pca9535_cfg, &pca9535_dev),
                        TAG, "add pca9535 device failed");

    /* Configure both ports as outputs */
    ESP_RETURN_ON_ERROR(pca9535_write_reg(pca9535_dev, 0x06, 0x00), TAG, "pca9535 config failed");
    ESP_RETURN_ON_ERROR(pca9535_write_reg(pca9535_dev, 0x07, 0x00), TAG, "pca9535 config failed");
    /* Set P10 (PWR_HOLD) and P13 (POWER_AMP_EN) to HIGH, others to LOW */
    ESP_RETURN_ON_ERROR(pca9535_write_reg(pca9535_dev, 0x02, 0x00), TAG, "pca9535 output failed");
    ESP_RETURN_ON_ERROR(pca9535_write_reg(pca9535_dev, 0x03, 0x09), TAG, "pca9535 output failed");

    /* Wait for the power rails to settle */
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "PCA9535 initialized, P10(PWR_HOLD)/P13(POWER_AMP_EN) set to HIGH");
    return ESP_OK;
}

/* 2. Initialize I2S controllers for ES8311 (TX, speaker) and ES7210 (RX, microphone) */
static esp_err_t i2s_driver_init(void)
{
    /* TX channel for ES8311 */
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(ES8311_I2S_NUM, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = ES8311_I2S_MCK_IO,
            .bclk = ES8311_I2S_BCK_IO,
            .ws = ES8311_I2S_WS_IO,
            .dout = ES8311_I2S_DO_IO,
            .din = I2S_GPIO_UNUSED, // ES8311 ADC output is not connected on this board
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    /* RX channel for ES7210, TDM 4-channel mode */
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(ES7210_I2S_NUM, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle));

    i2s_tdm_config_t rx_tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = EXAMPLE_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3,
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = I2S_TDM_AUTO_SLOT_NUM,
        },
        .gpio_cfg = {
            .mclk = ES7210_I2S_MCK_IO,
            .bclk = ES7210_I2S_BCK_IO,
            .ws = ES7210_I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = ES7210_I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(rx_handle, &rx_tdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    /* Reduce signal overshoot */
    gpio_set_drive_capability(ES8311_I2S_MCK_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES8311_I2S_BCK_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES8311_I2S_WS_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES8311_I2S_DO_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES7210_I2S_MCK_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES7210_I2S_BCK_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES7210_I2S_WS_IO, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ES7210_I2S_DI_IO, GPIO_DRIVE_CAP_1);

    ESP_LOGI(TAG, "I2S initialized (ES8311 TX on port %d, ES7210 RX on port %d)",
             ES8311_I2S_NUM, ES7210_I2S_NUM);
    return ESP_OK;
}

/* 3. Configure ES8311 codec for playback (also enables the power amplifier) */
static esp_err_t es8311_codec_init(void)
{
    /* Create control interface with the shared I2C bus handle */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(ctrl_if);

    /* Create data interface with I2S bus handle */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = ES8311_I2S_NUM,
        .rx_handle = NULL,
        .tx_handle = tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if);

    /* Create ES8311 interface handle */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    assert(gpio_if);
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .use_mclk = true,
        /* PA enable pin, driven high while playing (same as the factory firmware) */
        .pa_pin = EXAMPLE_PA_CTRL_IO,
        .pa_reverted = false,
        .hw_gain = {
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3,
        },
    };
    const audio_codec_if_t *es8311_if = es8311_codec_new(&es8311_cfg);
    assert(es8311_if);

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_if,
        .data_if = data_if,
    };
    play_dev = esp_codec_dev_new(&dev_cfg);
    assert(play_dev);

    /* Open the device: configures the ES8311 clock and enables the power amplifier */
    esp_codec_dev_sample_info_t sample_cfg = {
        .bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT,
        .channel = 2,
        .channel_mask = 0x03,
        .sample_rate = EXAMPLE_SAMPLE_RATE,
    };
    if (esp_codec_dev_open(play_dev, &sample_cfg) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Open es8311 play device failed");
        return ESP_FAIL;
    }

    if (esp_codec_dev_set_out_vol(play_dev, EXAMPLE_VOICE_VOLUME) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "set output volume failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ES8311 codec configured");
    return ESP_OK;
}

/* 4. Configure ES7210 codec for recording */
static esp_err_t es7210_codec_init(void)
{
    /* Create control interface with the shared I2C bus handle */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM,
        .addr = ES7210_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(ctrl_if);

    /* Create data interface with I2S bus handle */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = ES7210_I2S_NUM,
        .rx_handle = rx_handle,
        .tx_handle = NULL,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if);

    /* Enable all 4 microphones, the codec driver then works in TDM mode */
    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = ctrl_if,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4,
    };
    const audio_codec_if_t *es7210_if = es7210_codec_new(&es7210_cfg);
    assert(es7210_if);

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7210_if,
        .data_if = data_if,
    };
    record_dev = esp_codec_dev_new(&dev_cfg);
    assert(record_dev);

    /* Open the device: configures the ES7210 clock and enables the microphones */
    esp_codec_dev_sample_info_t sample_cfg = {
        .bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT,
        .channel = ES7210_TDM_CHANNELS,
        .channel_mask = 0x0F,
        .sample_rate = EXAMPLE_SAMPLE_RATE,
    };
    if (esp_codec_dev_open(record_dev, &sample_cfg) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Open es7210 record device failed");
        return ESP_FAIL;
    }

    if (esp_codec_dev_set_in_gain(record_dev, EXAMPLE_MIC_GAIN_DB) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "set mic gain failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ES7210 codec configured");
    return ESP_OK;
}

/* 5. Record and Play Task: Record 4-channel TDM data for specific seconds,
 * then extract channel 0/1 and play them back in stereo */
static void record_play_task(void *args)
{
    ESP_LOGI(TAG, "Allocating record buffer %d bytes and play buffer %d bytes in PSRAM...",
             RECORD_BUFFER_SIZE, PLAY_BUFFER_SIZE);

    // Allocate buffers in PSRAM to store the audio data
    int16_t *record_buf = heap_caps_malloc(RECORD_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int16_t *play_buf = heap_caps_malloc(PLAY_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!record_buf || !play_buf) {
        ESP_LOGE(TAG, "Failed to allocate buffers in PSRAM");
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_read = 0;
    size_t bytes_written = 0;
    const size_t record_frames = RECORD_BUFFER_SIZE / (sizeof(int16_t) * ES7210_TDM_CHANNELS);

    while (1) {
        // ----------------- Recording Phase (4-channel TDM) -----------------
        ESP_LOGI(TAG, "=== Start Recording for %d seconds ===", RECORD_TIME_SEC);
        size_t total_read = 0;

        // Read in small chunks to prevent watchdog timeout
        size_t chunk_size = 4096;
        while (total_read < RECORD_BUFFER_SIZE) {
            size_t to_read = RECORD_BUFFER_SIZE - total_read;
            if (to_read > chunk_size) {
                to_read = chunk_size;
            }

            if (i2s_channel_read(rx_handle, (uint8_t*)record_buf + total_read, to_read, &bytes_read, portMAX_DELAY) == ESP_OK) {
                total_read += bytes_read;
            } else {
                ESP_LOGE(TAG, "I2S read error");
                break;
            }
        }
        ESP_LOGI(TAG, "=== Recording Finished, total read: %d bytes ===", total_read);

        // Per-channel max amplitude, to tell whether the microphones captured anything
        int ch_max[ES7210_TDM_CHANNELS] = {0};
        for (size_t f = 0; f < record_frames; f++) {
            for (int ch = 0; ch < ES7210_TDM_CHANNELS; ch++) {
                int v = abs(record_buf[f * ES7210_TDM_CHANNELS + ch]);
                if (v > ch_max[ch]) {
                    ch_max[ch] = v;
                }
            }
        }
        ESP_LOGI(TAG, "Mic channel max amplitude: ch0=%d ch1=%d ch2=%d ch3=%d",
                 ch_max[0], ch_max[1], ch_max[2], ch_max[3]);

        // Extract channel 0 (left) and channel 1 (right) into the stereo play buffer
        for (size_t f = 0; f < record_frames; f++) {
            play_buf[f * 2]     = record_buf[f * ES7210_TDM_CHANNELS];
            play_buf[f * 2 + 1] = record_buf[f * ES7210_TDM_CHANNELS + 1];
        }

        // ----------------- Playing Phase (stereo) -----------------
        ESP_LOGI(TAG, "=== Start Playing for %d seconds ===", RECORD_TIME_SEC);
        size_t total_written = 0;

        while (total_written < PLAY_BUFFER_SIZE) {
            size_t to_write = PLAY_BUFFER_SIZE - total_written;
            if (to_write > chunk_size) {
                to_write = chunk_size;
            }

            if (i2s_channel_write(tx_handle, (uint8_t*)play_buf + total_written, to_write, &bytes_written, portMAX_DELAY) == ESP_OK) {
                total_written += bytes_written;
            } else {
                ESP_LOGE(TAG, "I2S write error");
                break;
            }
        }
        ESP_LOGI(TAG, "=== Playing Finished, total written: %d bytes ===", total_written);
    }
}

void app_main(void)
{
    printf("\n============================================\n");
    printf("   Record & Play Example (ES7210 + ES8311)  \n");
    printf("============================================\n\n");

    // 1. Power up the audio path first (3.3V rail hold + amplifier power)
    if (board_power_init() != ESP_OK) {
        ESP_LOGE(TAG, "board power init failed");
        abort();
    }

    // 2. Initialize I2S controllers for both codecs
    if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init failed");
        abort();
    }

    // 3. Configure ES8311 codec for playback (also enables the power amplifier)
    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    }

    // 4. Configure ES7210 codec for recording
    if (es7210_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es7210 codec init failed");
        abort();
    }

    // 5. Create Record and Play Loop Task
    xTaskCreate(record_play_task, "record_play_task", 4096, NULL, 5, NULL);
}
