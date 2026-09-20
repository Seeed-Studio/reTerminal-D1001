/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/pmu_reg.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#include "esp_lcd_jd9365_8.h"
#include "esp_lcd_touch_gsl3670.h"
#include "esp_io_expander_pca9535.h"

#include "esp32_p4_re_terminal_d1001.h"
#include "display.h"
#include "touch.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_pm.h"
#include "esp_sleep.h"
#include "iot_button.h"
#include "button_gpio.h"

#include "bsp_lsm6ds3.h"
#include "bsp_pcf8563.h"

static const char *TAG = "ESP32_P4_EV";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *disp_indev = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler

static bool i2c_0_initialized = false;
static bool i2c_1_initialized = false;

esp_io_expander_handle_t io_expander = NULL;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static i2c_master_bus_handle_t i2c_0_handle = NULL;  // I2C Handle
static i2c_master_bus_handle_t i2c_1_handle = NULL;  // I2C Handle
#endif

static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;

static const audio_codec_data_if_t *i2s_adc_data_if = NULL;
static const audio_codec_data_if_t *i2s_dac_data_if = NULL;

#define ADC_SAMPLE_NUM  16
#define AD_LOST_VAL     1
static bool adc_initialized = false;
static adc_oneshot_unit_handle_t adc_1_handle = NULL;
static int adc_raw_data[ADC_SAMPLE_NUM] = {0};
static int bsp_usb_volt = 0;
static int bsp_bat_volt = 0;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
static adc_cali_handle_t adc_1_cali_chan_1_handle = NULL;
static adc_cali_handle_t adc_1_cali_chan_2_handle = NULL;
#endif

sd_pwr_ctrl_handle_t sd_pwr_ctrl_handle = NULL;
static bool sd_pwr_ctrl_init = false;

#define BSP_BATTERY_CHARGE_PROTECT      0
#define BSP_BATTERY_CHARGE_TEMP_MAX     43
#define BSP_BATTERY_CHARGE_TEMP_MIN     2
static TaskHandle_t battery_manage_task;
static bool battery_task_init = false;

#define CHARGE_VOLT_HIGH    4150
#define CHARGE_VOLT_LOW     3800
static bool bat_chg_state = true;

// #define BATTERY_POINT       11
// #define PERCENT_STEP_MIN    10
// static int battery_level_percent_table[BATTERY_POINT] = {3262, 3389, 3488, 3586, 3658, 3711, 3762, 3819, 3872, 3916, 4047};
#define BATTERY_POINT       21
#define PERCENT_STEP_MIN    5
static int battery_level_percent_table[BATTERY_POINT] = {3262, 3390, 3467, 3554, 3619, 3659, 3686, 3710, 3731, 3752, 
                                                        3774, 3797, 3827, 3855, 3880, 3901, 3915, 3934, 3958, 3978, 4047};

esp_lcd_panel_handle_t disp_panel = NULL;

static TaskHandle_t backlight_auto_task;
static bool backlight_task_init = false;
static bool brightness_init = false;
static bool backlight_enable = true;
static bool backlight_auto = false;
static int backlight_default = 100;

static TaskHandle_t sd_card_m_task_handle;
static bool sd_card_insert = false;
static bool sd_mount_flag = false;

static uint32_t bsp_boot_count = 0;

#define LED_GREEN_DC_DEF    8
#define LED_RED_DC_DEF      20
#define LED_BLUE_DC_DEF     50

lsm6ds3_handle_t lsm6ds3;
pcf8563_handle_t pcf8563;

/* Can be used for `i2s_std_gpio_config_t` and/or `i2s_std_config_t` initialization */
#define BSP_I2S_ADC_GPIO_CFG       \
    {                              \
        .mclk = BSP_ADC_I2S_MCLK,  \
        .bclk = BSP_ADC_I2S_SCLK,  \
        .ws = BSP_ADC_I2S_LRLK,    \
        .din = BSP_ADC_I2S_SDIN,   \
        .dout = BSP_ADC_I2S_SDOUT, \
        .invert_flags = {          \
            .mclk_inv = false,     \
            .bclk_inv = false,     \
            .ws_inv = false,       \
        },                         \
    }
#define BSP_I2S_DAC_GPIO_CFG       \
    {                              \
        .mclk = BSP_DAC_I2S_MCLK,  \
        .bclk = BSP_DAC_I2S_SCLK,  \
        .ws = BSP_DAC_I2S_LRLK,    \
        .din = BSP_DAC_I2S_SDIN,   \
        .dout = BSP_DAC_I2S_SDOUT, \
        .invert_flags = {          \
            .mclk_inv = false,     \
            .bclk_inv = false,     \
            .ws_inv = false,       \
        },                         \
    }

/* This configuration is used by default in `bsp_extra_audio_init()` */
#define BSP_I2S_ADC_DUPLEX_MONO_CFG(_sample_rate)                                                     \
    {                                                                                                   \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                            \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_ADC_GPIO_CFG,                                                               \
    }

#define BSP_I2S_DAC_DUPLEX_MONO_CFG(_sample_rate)                                                     \
    {                                                                                                   \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                            \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_DAC_GPIO_CFG,                                                               \
    }

esp_err_t bsp_i2c_0_init(void)
{
    /* I2C was initialized before */
    if (i2c_0_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_0_SDA,
        .scl_io_num = BSP_I2C_0_SCL,
        .i2c_port = 0,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_0_handle));

    i2c_0_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_0_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_0_handle));
    i2c_0_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_0_get_handle(void)
{
    return i2c_0_handle;
}

esp_err_t bsp_i2c_1_init(void)
{
    /* I2C was initialized before */
    if (i2c_1_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_1_SDA,
        .scl_io_num = BSP_I2C_1_SCL,
        .i2c_port = 1,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_1_handle));

    i2c_1_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_1_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_1_handle));
    i2c_1_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_1_get_handle(void)
{
    return i2c_1_handle;
}

esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
// #ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
//         .format_if_mount_failed = true,
// #else
//         .format_if_mount_failed = false,
// #endif
        .format_if_mount_failed = true,
        .max_files = 16,
        .allocation_unit_size = 64 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    if (!sd_pwr_ctrl_init) {
        sd_pwr_ctrl_init = true;
        sd_pwr_ctrl_ldo_config_t ldo_config = {
            .ldo_chan_id = 4,
        };
        esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &sd_pwr_ctrl_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    host.pwr_ctrl_handle = sd_pwr_ctrl_handle;
    const sdmmc_slot_config_t slot_config = {
        /* SD card is connected to Slot 0 pins. Slot 0 uses IO MUX, so not specifying the pins here */
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = 0,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);

    // Reduce signal overshoot
    gpio_set_drive_capability(BSP_SD_CLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_SD_CMD, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_SD_D0, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_SD_D1, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_SD_D2, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_SD_D3, GPIO_DRIVE_CAP_1);

    return ret;
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_adc_config, const i2s_std_config_t *i2s_dac_config)
{
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg_dac = I2S_CHANNEL_DEFAULT_CONFIG(0, I2S_ROLE_MASTER);
    chan_cfg_dac.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_dac, &i2s_tx_chan, NULL));

    // Reduce signal overshoot
    gpio_set_drive_capability(BSP_DAC_I2S_MCLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_DAC_I2S_SCLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_DAC_I2S_LRLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_DAC_I2S_SDOUT, GPIO_DRIVE_CAP_1);

    i2s_chan_config_t chan_cfg_adc = I2S_CHANNEL_DEFAULT_CONFIG(1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_adc, NULL, &i2s_rx_chan));

    // Reduce signal overshoot
    gpio_set_drive_capability(BSP_ADC_I2S_MCLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_ADC_I2S_SCLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_ADC_I2S_LRLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_ADC_I2S_SDIN, GPIO_DRIVE_CAP_1);

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_dav_default = BSP_I2S_DAC_DUPLEX_MONO_CFG(16000);
    const i2s_std_config_t *p_i2s_dac_cfg = &std_cfg_dav_default;

    const i2s_std_config_t std_cfg_adc_default = BSP_I2S_ADC_DUPLEX_MONO_CFG(16000);
    const i2s_std_config_t *p_i2s_adc_cfg = &std_cfg_adc_default;
    
    if (i2s_dac_config != NULL) {
        p_i2s_dac_cfg = i2s_dac_config;
    }

    if (i2s_adc_config != NULL) {
        p_i2s_adc_cfg = i2s_adc_config;
    }

    if (i2s_tx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_dac_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
    }

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg =
            {
                .sample_rate_hz  = (uint32_t)16000,
                .clk_src         = I2S_CLK_SRC_DEFAULT,
                .ext_clk_freq_hz = 0,
                .mclk_multiple   = I2S_MCLK_MULTIPLE_256,
                .bclk_div        = 8,
            },
        .slot_cfg = {
                        .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                        .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                        .slot_mode      = I2S_SLOT_MODE_STEREO,
                        .slot_mask      = (I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
                        .ws_width       = I2S_TDM_AUTO_WS_WIDTH,
                        .ws_pol         = false,
                        .bit_shift      = true,
                        .left_align     = false,
                        .big_endian     = false,
                        .bit_order_lsb  = false,
                        .skip_mask      = false,
                        .total_slot     = I2S_TDM_AUTO_SLOT_NUM
                    },
        .gpio_cfg = BSP_I2S_ADC_GPIO_CFG,
    };

    // if (i2s_rx_chan != NULL) {
    //     ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx_chan, p_i2s_adc_cfg));
    //     ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));
    // }

    if (i2s_rx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_rx_chan, &tdm_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));
    }

    audio_codec_i2s_cfg_t i2s_cfg_dac = {
        .port = I2S_NUM_0,
        .rx_handle = NULL,
        .tx_handle = i2s_tx_chan,
    };
    i2s_dac_data_if = audio_codec_new_i2s_data(&i2s_cfg_dac);

    audio_codec_i2s_cfg_t i2s_cfg_adc = {
        .port = I2S_NUM_1,
        .rx_handle = i2s_rx_chan,
        .tx_handle = NULL,
    };
    i2s_adc_data_if = audio_codec_new_i2s_data(&i2s_cfg_adc);

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (i2s_dac_data_if == NULL) {
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL, NULL));
    }
    assert(i2s_dac_data_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = 1,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_1_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_TYPE_OUT,
        // .pa_pin = -1,
        .pa_pin = GPIO_NUM_53,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    assert(es8311_dev);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_dev,
        .data_if = i2s_dac_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_adc_data_if == NULL) {
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL, NULL));
    }
    assert(i2s_adc_data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = 1,
        .addr = ES7210_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_1_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if);

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4,
    };
    const audio_codec_if_t *es7210_dev = es7210_codec_new(&es7210_cfg);
    assert(es7210_dev);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7210_dev,
        .data_if = i2s_adc_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

// Bit number used to represent command and parameter
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_channel_config_t RGB_red_channel = {
        .gpio_num = BSP_LED_R,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 100,
        .hpoint = 0
    };
    const ledc_channel_config_t RGB_green_channel = {
        .gpio_num = BSP_LED_G,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 100,
        .hpoint = 0
    };
    const ledc_channel_config_t RGB_blue_channel = {
        .gpio_num = BSP_LED_B,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 4,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 100,
        .hpoint = 0
    };
    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&RGB_red_channel));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&RGB_green_channel));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&RGB_blue_channel));
    brightness_init = true;
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_init) {
        if (brightness_percent > 100) {
            brightness_percent = 100;
        }
        if (brightness_percent < 0) {
            brightness_percent = 0;
        }
        if (brightness_percent) {
            backlight_default = brightness_percent;
        }
        
        ESP_LOGD(TAG, "Setting LCD backlight: %d%%", brightness_percent);
        uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
        BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
        BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(backlight_default);
}

esp_err_t bsp_rgb_led_duty_set(uint8_t rgb, int percent)
{
    char str[4] = {'r','g','b',0};
    if(rgb > 2) return ESP_FAIL;
    if (brightness_init) {
        if (percent > 100) {
            percent = 100;
        }
        if (percent < 0) {
            percent = 0;
        }

        ESP_LOGI(TAG, "Setting RGB led: %c, %d%%", str[rgb], percent);
        uint32_t duty_cycle = (1024 * (100 - percent)) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
        BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, rgb + 2, duty_cycle));
        BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, rgb + 2));
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

static esp_err_t bsp_enable_dsi_phy_power(void)
{
#if BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), TAG, "Acquire LDO channel for DPHY failed");
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif // BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0

    return ESP_OK;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    bsp_lcd_handles_t handles;
    ret = bsp_display_new_with_handles(config, &handles);

    *ret_panel = handles.panel;
    *ret_io = handles.io;

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = BSP_LCD_MIPI_DSI_LANE_NUM,
        .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // according to the LCD spec
        .lcd_param_bits = 8, // according to the LCD spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

    // create JD9365 control panel
    ESP_LOGI(TAG, "Install JD9365 LCD control panel");
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_8_800_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_FMT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_8_800_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_FMT_RGB565);
#endif
    dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;

    jd9365_8_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = BSP_LCD_MIPI_DSI_LANE_NUM,
        },
    };
    const esp_lcd_panel_dev_config_t lcd_dev_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .bits_per_pixel = 24,
#else
        .bits_per_pixel = 16,
#endif
        .vendor_config = &vendor_config,
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_jd9365_8(io, &lcd_dev_config, &disp_panel), err, TAG, "New LCD panel JD9365 failed");
    ESP_GOTO_ON_ERROR(esp_lcd_dpi_panel_enable_dma2d(disp_panel), err, TAG, "Enable DMA2D for LCD panel failed");

    ESP_LOGI(TAG, "LCD panel hw reset");
    esp_io_expander_set_level(io_expander, BSP_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    esp_io_expander_set_level(io_expander, BSP_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, BSP_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(disp_panel, true), err, TAG, "LCD panel ON failed");

    /* Return all handles */
    if (ret_handles) {
        ret_handles->io = io;
        ret_handles->mipi_dsi_bus = mipi_dsi_bus;
        ret_handles->panel = disp_panel;
        ret_handles->control = NULL;
    }

    ESP_LOGI(TAG, "Display initialized");

    return ret;

err:
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = 12,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GSL3670_CONFIG();
    tp_io_config.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_0_handle, &tp_io_config, &tp_io_handle), TAG, "");

    // ESP_LOGI(TAG, "LCD touch hw reset");
    // esp_io_expander_set_level(io_expander, BSP_LCD_TOUCH_RST, 1);
    // vTaskDelay(pdMS_TO_TICKS(5));
    // esp_io_expander_set_level(io_expander, BSP_LCD_TOUCH_RST, 0);
    // vTaskDelay(pdMS_TO_TICKS(10));
    // esp_io_expander_set_level(io_expander, BSP_LCD_TOUCH_RST, 1);
    // vTaskDelay(pdMS_TO_TICKS(50));

    return esp_lcd_touch_new_i2c_gsl3670(tp_io_handle, &tp_cfg, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
#if LVGL_VERSION_MAJOR >= 9
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .color_format = LV_COLOR_FORMAT_RGB888,
#else
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
#endif
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .sw_rotate = false,                /* Avoid tearing is not supported for SW rotation */
#else
            .sw_rotate = cfg->flags.sw_rotate, /* Only SW rotation is supported for 90° and 270° */
#endif
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
        }
    };

    const lvgl_port_display_dsi_cfg_t dpi_cfg = {
        .flags = {
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };

    return lvgl_port_add_disp_dsi(&disp_cfg, &dpi_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    esp_lcd_touch_handle_t tp;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
            .buff_dma = false,
#else
            .buff_dma = true,
#endif
            .buff_spiram = false,
            .sw_rotate = true,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

static esp_err_t bsp_adc_init(void)
{
    esp_err_t ret = ESP_OK;

    if( adc_initialized == false ) {
        adc_initialized = true;

        adc_oneshot_unit_init_cfg_t adc_config = {
            .unit_id  = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };

        ret = adc_oneshot_new_unit(&adc_config, &adc_1_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc add new fail");
            return ESP_FAIL;
        }

        adc_oneshot_chan_cfg_t chan_config = {
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };

        ret = adc_oneshot_config_channel(adc_1_handle, ADC_CHANNEL_1, &chan_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc config chan 1 fail");
            return ESP_FAIL;
        }

        ret = adc_oneshot_config_channel(adc_1_handle, ADC_CHANNEL_2, &chan_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc config chan 2 fail");
            return ESP_FAIL;
        }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_config_1 = {
                .unit_id = ADC_UNIT_1,
                .chan = ADC_CHANNEL_1,
                .atten = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config_1, &adc_1_cali_chan_1_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc cali create chan 1 fail");
            return ESP_FAIL;
        }

        adc_cali_curve_fitting_config_t cali_config_2 = {
                .unit_id = ADC_UNIT_1,
                .chan = ADC_CHANNEL_2,
                .atten = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config_2, &adc_1_cali_chan_2_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc cali create chan 2 fail");
            return ESP_FAIL;
        }
#endif
    }

    return ESP_OK;
}

static int bsp_adc_read(adc_channel_t channel)
{
    int voltage = 0;
    uint32_t sum = 0;
    int volt_temp = 0;
    uint16_t temp_val = 0;

    for (uint16_t i = 0; i < ADC_SAMPLE_NUM; i++) {
        adc_raw_data[i] = 0;
    }
    for (uint16_t i = 0; i < ADC_SAMPLE_NUM; i++) {
        adc_oneshot_read(adc_1_handle, channel, &adc_raw_data[i]);
    }

    for (uint16_t i = 0; i < ADC_SAMPLE_NUM - 1; i++) {
        for (uint16_t j = i + 1; j < ADC_SAMPLE_NUM; j++) {
            if (adc_raw_data[i] > adc_raw_data[j]) {
                temp_val = adc_raw_data[i];
                adc_raw_data[i] = adc_raw_data[j];
                adc_raw_data[j] = temp_val;
            }
        }
    }
    for (uint16_t i = AD_LOST_VAL; i < ADC_SAMPLE_NUM - AD_LOST_VAL; i++) {
        sum += adc_raw_data[i];
    }
    volt_temp = sum / (ADC_SAMPLE_NUM - 2 * AD_LOST_VAL);
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (channel == ADC_CHANNEL_1) {
        adc_cali_raw_to_voltage(adc_1_cali_chan_1_handle, volt_temp, &voltage);
    } else if (channel == ADC_CHANNEL_2) {
        adc_cali_raw_to_voltage(adc_1_cali_chan_2_handle, volt_temp, &voltage);
    }
#else
    voltage = volt_temp;
#endif

    return voltage;
}

int bsp_battery_voltage_read(void)
{
    return bsp_bat_volt;
}

static uint8_t vol_to_percentage(uint16_t voltage)
{
    if (voltage < battery_level_percent_table[0]) {
        return 0;
    }

    for (uint8_t i = 0; i < BATTERY_POINT; i++) {
        if (voltage < battery_level_percent_table[i]) {
            return (i - 1) * PERCENT_STEP_MIN + (PERCENT_STEP_MIN * (int)(voltage - battery_level_percent_table[i-1])) / 
                (int)(battery_level_percent_table[i] - battery_level_percent_table[i-1]);
        }
    }

    return 100;
}

int bsp_battery_percent_read(void)
{
    uint8_t per = vol_to_percentage(bsp_bat_volt);
    if (per == 0) {
        return 0;
    } else if (per > 0 && per < 5) {
        return 1;
    } else {
        return (per / 5) * 5;
    }
}

int bsp_usb_voltage_read(void)
{
    return bsp_usb_volt;
}

bool bsp_battery_charge_status_read(void)
{
    int status = 0;
    status = gpio_get_level(BSP_BAT_CHARGE_STATE);
    return (status? false : true);
}

static bool bsp_chg_error = false;
static uint32_t chg_high_ms = 0;
static uint32_t chg_low_ms = 0;

static void IRAM_ATTR charge_status_isr_handler(void* arg)
{
    int status = gpio_get_level(BSP_BAT_CHARGE_STATE);
    if (status) {
        chg_high_ms = (uint32_t)(esp_timer_get_time() / 1000);
    } else {
        chg_low_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }
    if (chg_high_ms && chg_low_ms && (chg_high_ms - chg_low_ms) < 2000) {
        bsp_chg_error = true;
    }
}

static void bsp_battery_charge_en(bool en)
{
    if (en) {
        if (!bat_chg_state) {
            bat_chg_state = true;
            esp_io_expander_set_level(io_expander, BSP_BAT_CHARGE_EN, 0); // enable battery charge
        }
    } else {
        if (bat_chg_state) {
            bat_chg_state = false;
            esp_io_expander_set_level(io_expander, BSP_BAT_CHARGE_EN, 1); // disable battery charge
        }
    }
}

#define VOLT_AVG_MAX    60
static void bsp_battery_charge_task(void *arg)
{
    uint8_t log_err_cnt = 0;
    int bsp_usb_volt_cur = 0;
    int bsp_bat_volt_cur = 0;
    int volt[VOLT_AVG_MAX] = { 0 }, index = 0, count = 0; 
    int32_t volt_sum = 0, volt_avg = 0;
    while (1) {
        bsp_usb_volt = bsp_adc_read(ADC_CHANNEL_1) * 2;
        bsp_bat_volt_cur = bsp_adc_read(ADC_CHANNEL_2) * 2;

        volt[index ++] = bsp_bat_volt_cur;
        if (index >= VOLT_AVG_MAX) {
            index = 0;
        }
        if (count < VOLT_AVG_MAX) {
            count ++;
        }
        volt_sum = 0;
        for (int16_t i = 0; i < count; i++) {
            volt_sum += volt[i];
        }
        volt_avg = volt_sum / count;
        bsp_bat_volt_cur = volt_avg;

        if (bsp_bat_volt == 0) {
            bsp_bat_volt = bsp_bat_volt_cur;
        }

        if (bsp_bat_volt_cur > CHARGE_VOLT_HIGH) {
            if (bat_chg_state) {
                bsp_battery_charge_en(false);
                ESP_LOGW(TAG, "battery high, disable charge");
            }
        } else if (bsp_bat_volt_cur < CHARGE_VOLT_LOW) {
            if (!bat_chg_state) {
                bsp_battery_charge_en(true);
                ESP_LOGW(TAG, "battery low, enable charge");
            }
        } else {
            if (abs(bsp_usb_volt - bsp_usb_volt_cur) > 2000)
            {
                bsp_usb_volt_cur = bsp_usb_volt;
                if (bsp_usb_volt > 2000) {
                    bsp_battery_charge_en(true);
                    ESP_LOGW(TAG, "usb insert, enable charge");
                }
            }
        }        

        if (bsp_usb_volt > 4000) {
            bsp_bat_volt = bsp_bat_volt_cur;
            if (!bsp_chg_error) {
                log_err_cnt = 0;
                int charge = gpio_get_level(BSP_BAT_CHARGE_STATE);
                if (charge) {
                    // high: charge done
                } else {
                    // low: charging
                }
            } else {
                bsp_chg_error = false;
                log_err_cnt ++;
                if (log_err_cnt >= 5) {
                    log_err_cnt = 0;
                    ESP_LOGE(TAG, "battery charge error!!!");
                }
            }
        } else {
            if (bsp_bat_volt_cur < bsp_bat_volt) {
                bsp_bat_volt = bsp_bat_volt_cur;
            }
        }

#if BSP_BATTERY_CHARGE_PROTECT
        float temp = 0;
        bsp_lsm6ds3_read_temp(&lsm6ds3, &temp);
        if (temp <= BSP_BATTERY_CHARGE_TEMP_MAX && temp >= BSP_BATTERY_CHARGE_TEMP_MIN) {
            bsp_battery_charge_en(true);
            
        } else {
            bsp_battery_charge_en(false);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(1000));
        bsp_boot_count += 1;
    }
}

esp_err_t bsp_battery_manage_start(void)
{
    esp_err_t ret = ESP_OK;
    if (!battery_task_init) {
        battery_task_init = true;

        const gpio_config_t int_gpio_config = {
            .pull_up_en = 1,
            .pull_down_en = 0,
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_ANYEDGE,
            .pin_bit_mask = 1ULL << BSP_BAT_CHARGE_STATE,
        };
        ret = gpio_config(&int_gpio_config);
        gpio_install_isr_service(0);
        // do not force to run in IRAM, but compatible with ESP_INTR_FLAG_IRAM
        gpio_isr_handler_add(BSP_BAT_CHARGE_STATE, charge_status_isr_handler, (void *)BSP_BAT_CHARGE_STATE);

        gpio_config_t io_in_conf = {
            .pull_up_en = 1,
            .pull_down_en = 0,
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_DISABLE,
        };
        io_in_conf.pin_bit_mask = 1ULL << BSP_BAT_VSYS_PG,
        gpio_config(&io_in_conf);

        if (xTaskCreate(bsp_battery_charge_task, "bat_mng", 3072, NULL, 1, &battery_manage_task) != pdTRUE) {
            ESP_LOGE(TAG, "Creating battery manage task failed");
            abort();
        }
    }

    return ESP_OK;
}

static void IRAM_ATTR bsp_sdcard_detect_isr_handler(void* arg)
{
    int status = gpio_get_level(BSP_SD_DETECT); // high: remove, low: insert
    if (status) {
        sd_card_insert = false;
    } else {
        sd_card_insert = true;
    }
    if (sd_card_m_task_handle) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(sd_card_m_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void bsp_sd_card_auto_mount_task(void *arg)
{
    esp_err_t ret = ESP_OK;
    while (1) {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        if (sd_card_insert) {
            if (!sd_mount_flag) {
                ret = bsp_sdcard_mount();
                if (ret != ESP_OK) {
                    sd_mount_flag = false;
                    ESP_LOGE(TAG, "bsp sdcard mount fail");
                } else {
                    sd_mount_flag = true;
                    ESP_LOGW(TAG, "bsp sdcard mount ok");
                }
            }
        } else {
            if (sd_mount_flag) {
                ret = bsp_sdcard_unmount();
                if (ret != ESP_OK) {
                    sd_mount_flag = true;
                    ESP_LOGE(TAG, "bsp sdcard unmount fail");
                } else {
                    sd_mount_flag = false;
                    ESP_LOGW(TAG, "bsp sdcard unmount ok");
                }
            }
        }
    }
}

esp_err_t bsp_sd_card_manage_start(bool cmd)
{
    esp_err_t ret = ESP_OK;

    const gpio_config_t int_gpio_config = {
        .pull_up_en = 0,
        .pull_down_en = 0,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = 1ULL << BSP_SD_DETECT,
    };
    ret = gpio_config(&int_gpio_config);
    gpio_install_isr_service(0);
    // do not force to run in IRAM, but compatible with ESP_INTR_FLAG_IRAM
    gpio_isr_handler_add(BSP_SD_DETECT, bsp_sdcard_detect_isr_handler, (void *)BSP_SD_DETECT);

    if (cmd) {
        if (gpio_get_level(BSP_SD_DETECT) == 0) {
            sd_card_insert = true;
            ret = bsp_sdcard_mount();
            if (ret == ESP_OK) {
                sd_mount_flag = true;
                ESP_LOGW(TAG, "bsp sdcard mount ok");
            } else {
                sd_mount_flag = false;
                ESP_LOGE(TAG, "bsp sdcard mount fail");
            }
        }
    }

    xTaskCreate(bsp_sd_card_auto_mount_task, "sd-card", 4096, NULL, 1, &sd_card_m_task_handle);

    return ret;
}

bool bsp_sd_card_get_mount_state(void)
{
    return sd_mount_flag;
}

bool bsp_backlight_get_enable_state(void)
{
    return backlight_enable;
}

static void bsp_button_single_click_event_cb(void *arg, void *data)
{
    esp_err_t ret = ESP_OK;
    if (bsp_boot_count > 6) { // Prevent accidental triggering during startup
        if (backlight_enable) {
            backlight_enable = false;
            bsp_display_brightness_set(0);
            ESP_LOGI(TAG, "display brightness: 0");
            if (disp_panel) {
                ret = esp_lcd_panel_disp_on_off(disp_panel, false);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "display panel trun off");
                }
            }
        } else {
            backlight_enable = true;
            bsp_display_brightness_set(backlight_default);
            ESP_LOGI(TAG, "display brightness: %d", backlight_default);
            if (disp_panel) {
                ret = esp_lcd_panel_disp_on_off(disp_panel, true);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "display panel trun on");
                }
            }
        }
    }
}

static void bsp_button_long_press_event_cb(void *arg, void *data)
{
    // TODO
    // need to save some data on here

    ESP_LOGI(TAG, "system power off");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_io_expander_set_level(io_expander, BSP_LTE_PWR_EN, 0); // need trun off lte power first
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_io_expander_set_level(io_expander, BSP_PWR_HOLD, 0);
}

void bsp_led_red_set(bool cmd)
{
    // gpio_set_level(BSP_LED_R, cmd? false : true);
    if(cmd) bsp_rgb_led_duty_set(0, LED_RED_DC_DEF);
    else bsp_rgb_led_duty_set(0, 0);
}

void bsp_led_green_set(bool cmd)
{
    // gpio_set_level(BSP_LED_G, cmd? false : true);
    if(cmd) bsp_rgb_led_duty_set(1, LED_GREEN_DC_DEF);
    else bsp_rgb_led_duty_set(1, 0);
}

void bsp_led_blue_set(bool cmd)
{
    // gpio_set_level(BSP_LED_B, cmd? false : true);
    if(cmd) bsp_rgb_led_duty_set(2, LED_BLUE_DC_DEF);
    else bsp_rgb_led_duty_set(2, 0);
}

void bsp_power_off(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_io_expander_set_level(io_expander, BSP_LTE_PWR_EN, 0); // need trun off lte power first
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_io_expander_set_level(io_expander, BSP_PWR_HOLD, 0);
}

esp_err_t bsp_power_init(void)
{
    SET_PERI_REG_BITS(PMU_HP_ACTIVE_BIAS_REG, PMU_HP_ACTIVE_DCM_VSET, 26, PMU_HP_ACTIVE_DCM_VSET_S);

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_0_init());
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_1_init());
    BSP_ERROR_CHECK_RETURN_ERR(bsp_adc_init());

    /* Initilize IO expander */
    esp_err_t ret = ESP_OK;
    ret = esp_io_expander_new_i2c_pca9535(i2c_1_handle, ESP_IO_EXPANDER_I2C_PCA9535_ADDRESS_000, &io_expander);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander init fail");
        BSP_ERROR_CHECK_RETURN_ERR(ret);
    } else {
        ESP_LOGI(TAG, "io expander init ok");
    }
    esp_io_expander_set_dir(io_expander, 0xffff, IO_EXPANDER_OUTPUT);

    esp_io_expander_set_level(io_expander, BSP_PWR_HOLD, 1); // turn on vdd_3v3
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_io_expander_set_level(io_expander, BSP_LCD_BACKLIGHT_EN, 1); // turn on backlight power
    esp_io_expander_set_level(io_expander, BPS_LCD_PWR_EN, 1); // turn on display power
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_io_expander_set_level(io_expander, BSP_LCD_RST, 1);

    esp_io_expander_set_level(io_expander, BSP_CAM_EN, 1); // turn on camera power
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_io_expander_set_level(io_expander, BSP_CAM_PWDN, 1); // 
    esp_io_expander_set_level(io_expander, BSP_CAM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, BSP_CAM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, BSP_CAM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_io_expander_set_level(io_expander, BSP_LTE_PWR_EN, 1); // trun on lte power
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_io_expander_set_level(io_expander, BSP_LTE_AIRPLANE_MODE, 1);
    esp_io_expander_set_level(io_expander, BSP_LTE_DTR, 1);
    esp_io_expander_set_level(io_expander, BSP_LTE_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, BSP_LTE_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, BSP_LTE_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_io_expander_set_level(io_expander, BSP_BAT_READ_EN, 1); // turn on battery read
    esp_io_expander_set_level(io_expander, BSP_BAT_CHARGE_EN, 0); // 1: disable battery charge, 0: enable battery charge

    esp_io_expander_set_level(io_expander, BSP_POWER_AMP_EN, 1); // turn on amp power

    gpio_config_t io_in_conf = {
        .pull_up_en = 1,
        .pull_down_en = 0,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };

    io_in_conf.pin_bit_mask = 1ULL << BSP_BAT_CHARGE_STATE,
    gpio_config(&io_in_conf);

    gpio_config_t io_out_conf = {
        .mode = GPIO_MODE_OUTPUT,
    };

    // io_out_conf.pin_bit_mask = 1ULL << BSP_LED_R;
    // gpio_config(&io_out_conf);
    // gpio_set_level(BSP_LED_R, 0); // trun off led red

    // io_out_conf.pin_bit_mask = 1ULL << BSP_LED_G;
    // gpio_config(&io_out_conf);
    // gpio_set_level(BSP_LED_G, 0); // trun off led green

    // io_out_conf.pin_bit_mask = 1ULL << BSP_LED_B;
    // gpio_config(&io_out_conf);
    // gpio_set_level(BSP_LED_B, 0); // trun off led blue

    // Reduce signal overshoot
    gpio_set_drive_capability(BSP_WIFI_SDIO_CLK, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_WIFI_SDIO_CMD, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_WIFI_SDIO_D0, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_WIFI_SDIO_D1, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_WIFI_SDIO_D2, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(BSP_WIFI_SDIO_D3, GPIO_DRIVE_CAP_1);

    io_in_conf.pull_up_en = 0;
    io_in_conf.pull_down_en = 0;
    io_in_conf.pin_bit_mask = 1ULL << BSP_WIFI_TXD0,
    gpio_config(&io_in_conf);
    io_in_conf.pin_bit_mask = 1ULL << BSP_WIFI_RXD0,
    gpio_config(&io_in_conf);
    io_in_conf.pin_bit_mask = 1ULL << BSP_WIFI_BOOT,
    gpio_config(&io_in_conf);

    // if (!sd_pwr_ctrl_init) {
    //     sd_pwr_ctrl_init = true;
    //     sd_pwr_ctrl_ldo_config_t ldo_config = {
    //         .ldo_chan_id = 4,
    //     };
    //     esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &sd_pwr_ctrl_handle);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
    bsp_sdcard_mount(); // trun on core_vdd_5

    io_out_conf.pin_bit_mask = 1ULL << BSP_SD_PWR_EN;
    gpio_config(&io_out_conf);
    gpio_set_level(BSP_SD_PWR_EN, 0); // power off sd card
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BSP_SD_PWR_EN, 1); // power on sd card

    bsp_new_i2c_pcf8563(i2c_1_handle, PCF8563_I2C_ADDRESS, &pcf8563);
    bsp_new_i2c_lsm6ds3(i2c_1_handle, LSM6DS3_I2C_ADDRESS, &lsm6ds3);

    // rtc_datatime_t datetime = {
    //     .year = 2025, .month = 1, .day = 1,
    //     .hour = 0, .minute = 0, .second = 0,
    //     .week = 3,
    // };
    // ret = bsp_pcf8563_set_datatime(&pcf8563, &datetime);
    // BSP_ERROR_CHECK_RETURN_ERR(ret);

    ret = bsp_lsm6ds3_config_default(&lsm6ds3);
    // BSP_ERROR_CHECK_RETURN_ERR(ret);

    bsp_battery_manage_start();
    bsp_sd_card_manage_start(true);
    
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = BSP_BUTTON_IN,
        .active_level = 1,
        .disable_pull = true,
        .enable_power_save = false,
    };
    button_handle_t bsp_btn;
    iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &bsp_btn);
    iot_button_set_param(bsp_btn, BUTTON_SHORT_PRESS_TIME_MS, (void *)100);
    iot_button_set_param(bsp_btn, BUTTON_LONG_PRESS_TIME_MS, (void *)3000);
    iot_button_register_cb(bsp_btn, BUTTON_SINGLE_CLICK, NULL, bsp_button_single_click_event_cb, NULL);
    iot_button_register_cb(bsp_btn, BUTTON_LONG_PRESS_HOLD, NULL, bsp_button_long_press_event_cb, NULL);

    return ESP_OK;
}
