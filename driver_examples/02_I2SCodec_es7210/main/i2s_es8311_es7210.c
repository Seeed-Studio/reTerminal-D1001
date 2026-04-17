
// ==================== Record and Play Loop Test (ES8311 + ES7210) ====================

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_check.h"
#include "es8311.h"
#include "es7210.h"
#include "example_config.h"

static const char *TAG = "record_play_es7210_es8311";

/* Global I2S channel handles */
static i2s_chan_handle_t tx_handle_es8311 = NULL;
static i2s_chan_handle_t rx_handle_es7210 = NULL;

/* 1. Initialize PCA9535RGER */
static esp_err_t pca9535_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(1, PCA9535_I2C_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

static void pca9535_init(void)
{
    int i2c_master_port = 1; // Use I2C_NUM_1
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MISC_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MISC_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    pca9535_write_reg(0x06, 0x00); 
    pca9535_write_reg(0x07, 0x00); 
    pca9535_write_reg(0x02, 0x00); 
    pca9535_write_reg(0x03, 0x08); // Set P13 (EXP_GPO11) to HIGH

    ESP_LOGI(TAG, "PCA9535 initialized, P13 set to HIGH");
}

/* 2. Initialize I2S controller for ES8311 (Speaker) */
static esp_err_t i2s_driver_init_es8311(void)
{
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true; 
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle_es8311, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED, // No need for RX
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_es8311, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_es8311));
    ESP_LOGI(TAG, "ES8311 I2S initialized");
    return ESP_OK;
}

/* 3. Configure ES8311 Codec via I2C */
static esp_err_t es8311_codec_init(void)
{
    /* Initialize I2C0 for audio codec */
    const i2c_config_t es_i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM, &es_i2c_cfg);
    i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);

    /* Initialize ES8311 */
    es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
    ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
        .sample_frequency = EXAMPLE_SAMPLE_RATE
    };

    ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE, EXAMPLE_SAMPLE_RATE), TAG, "set es8311 sample frequency failed");
    ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");
    ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");
    ESP_LOGI(TAG, "ES8311 codec configured");
    return ESP_OK;
}

/* 4. Initialize I2S controller for ES7210 (Microphone) */
static esp_err_t i2s_driver_init_es7210(void)
{
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(ES7210_I2S_NUM, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle_es7210));

    // Use standard mode (Stereo) to ensure I2S left/right channel timing alignment 
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = ES7210_I2S_MCK_IO,
            .bclk = ES7210_I2S_BCK_IO,
            .ws = ES7210_I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED, // No need for TX
            .din = ES7210_I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_es7210, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_es7210));
    ESP_LOGI(TAG, "ES7210 I2S initialized in STD mode (Stereo)");
    return ESP_OK;
}

/* 5. Configure ES7210 Codec via I2C */
static esp_err_t es7210_codec_init(void)
{
    es7210_dev_handle_t es7210_handle = NULL;
    es7210_i2c_config_t i2c_conf = {
        .i2c_port = I2C_NUM,         // Share I2C0 with ES8311
        .i2c_addr = ES7210_ADDRRES_00 // Default I2C address 0x40
    };
    ESP_RETURN_ON_ERROR(es7210_new_codec(&i2c_conf, &es7210_handle), TAG, "es7210 create failed");

    es7210_codec_config_t codec_conf = {
        .sample_rate_hz = EXAMPLE_SAMPLE_RATE,
        .mclk_ratio = EXAMPLE_MCLK_MULTIPLE,
        .i2s_format = ES7210_I2S_FMT_I2S,
        .bit_width = ES7210_I2S_BITS_16B,
        .mic_bias = ES7210_MIC_BIAS_2V87,
        .mic_gain = ES7210_MIC_GAIN_24DB, // Lower gain to prevent distortion for near-field voice
        .flags.tdm_enable = false // Disable TDM mode, use standard I2S
    };
    ESP_RETURN_ON_ERROR(es7210_config_codec(es7210_handle, &codec_conf), TAG, "es7210 config failed");
    ESP_RETURN_ON_ERROR(es7210_config_volume(es7210_handle, 0), TAG, "es7210 volume config failed");
    ESP_LOGI(TAG, "ES7210 codec configured");
    return ESP_OK;
}

/* 6. Record and Play Task: Record for specific seconds, then play it back */
static void record_play_task(void *args)
{
    ESP_LOGI(TAG, "Allocating %d bytes in PSRAM for recording buffer...", RECORD_BUFFER_SIZE);
    
    // Allocate a large buffer in PSRAM to store the audio data
    int16_t *record_buf = heap_caps_malloc(RECORD_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!record_buf) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes in PSRAM for recording", RECORD_BUFFER_SIZE);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Successfully allocated %d bytes in PSRAM", RECORD_BUFFER_SIZE);

    size_t bytes_read = 0;
    size_t bytes_written = 0;

    while (1) {
        // ----------------- Recording Phase -----------------
        ESP_LOGI(TAG, "=== Start Recording for %d seconds ===", RECORD_TIME_SEC);
        size_t total_read = 0;
        
        // Read in small chunks to prevent watchdog timeout
        size_t chunk_size = 4096;
        while (total_read < RECORD_BUFFER_SIZE) {
            size_t to_read = RECORD_BUFFER_SIZE - total_read;
            if (to_read > chunk_size) {
                to_read = chunk_size;
            }
            
            if (i2s_channel_read(rx_handle_es7210, (uint8_t*)record_buf + total_read, to_read, &bytes_read, portMAX_DELAY) == ESP_OK) {
                total_read += bytes_read;
            } else {
                ESP_LOGE(TAG, "I2S read error");
                break;
            }
        }
        ESP_LOGI(TAG, "=== Recording Finished, total read: %d bytes ===", total_read);

        // ----------------- Playing Phase -----------------
        ESP_LOGI(TAG, "=== Start Playing for %d seconds ===", RECORD_TIME_SEC);
        size_t total_written = 0;
        
        while (total_written < total_read) {
            size_t to_write = total_read - total_written;
            if (to_write > chunk_size) {
                to_write = chunk_size;
            }
            
            if (i2s_channel_write(tx_handle_es8311, (uint8_t*)record_buf + total_written, to_write, &bytes_written, portMAX_DELAY) == ESP_OK) {
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

    // 1. Initialize PCA9535RGER IO expander, set EXP_GPO11(P13) to HIGH
    pca9535_init();

    // 2. Initialize internal I2S controller for ES8311
    if (i2s_driver_init_es8311() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init es8311 failed");
        abort();
    }

    // 3. Configure ES8311 Codec via I2C
    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    }

    // 4. Initialize internal I2S controller for ES7210
    if (i2s_driver_init_es7210() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init es7210 failed");
        abort();
    }

    // 5. Configure ES7210 Codec via I2C
    if (es7210_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es7210 codec init failed");
        abort();
    }

    // 6. Create Record and Play Loop Task
    xTaskCreatePinnedToCore(record_play_task, "record_play_task", 32768, NULL, 5, NULL, 0);
}
