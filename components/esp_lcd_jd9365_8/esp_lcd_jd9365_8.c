/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"

#if SOC_MIPI_DSI_SUPPORTED
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_vendor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_lcd_jd9365_8.h"

#define JD9356_8_TEST_PATTERN   0

#define JD9365_8_CMD_PAGE       (0xE0)
#define JD9365_8_PAGE_USER      (0x00)

#define JD9365_8_CMD_DSI_INT0   (0x80)
#define JD9365_8_DSI_1_LANE     (0x00)
#define JD9365_8_DSI_2_LANE     (0x01)
#define JD9365_8_DSI_3_LANE     (0x10)
#define JD9365_8_DSI_4_LANE     (0x11)

#define JD9365_8_CMD_GS_BIT     (1 << 0)
#define JD9365_8_CMD_SS_BIT     (1 << 1)
#define JD9365_8_CMD_MV_BIT     (1 << 5)

typedef struct
{
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save surrent value of LCD_CMD_COLMOD register
    const jd9365_8_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    uint8_t lane_num;
    struct
    {
        unsigned int reset_level : 1;
    } flags;
    // To save the original functions of MIPI DPI panel
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} jd9365_8_panel_t;

static const char *TAG = "jd9365_8";

static esp_err_t panel_jd9365_8_del(esp_lcd_panel_t *panel);
static esp_err_t panel_jd9365_8_init(esp_lcd_panel_t *panel);
static esp_err_t panel_jd9365_8_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_jd9365_8_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_jd9365_8_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_jd9365_8_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_jd9365_8_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_jd9365_8_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

esp_err_t esp_lcd_new_panel_jd9365_8(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", ESP_LCD_JD9365_8_VER_MAJOR, ESP_LCD_JD9365_8_VER_MINOR,
             ESP_LCD_JD9365_8_VER_PATCH);
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid arguments");
    jd9365_8_vendor_config_t *vendor_config = (jd9365_8_vendor_config_t *)panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config && vendor_config->mipi_config.dsi_bus, ESP_ERR_INVALID_ARG, TAG,
                        "invalid vendor config");

    esp_err_t ret = ESP_OK;
    jd9365_8_panel_t *jd9365_8 = (jd9365_8_panel_t *)calloc(1, sizeof(jd9365_8_panel_t));
    ESP_RETURN_ON_FALSE(jd9365_8, ESP_ERR_NO_MEM, TAG, "no mem for jd9365_8 panel");

    if (panel_dev_config->reset_gpio_num >= 0)
    {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order)
    {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        jd9365_8->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        jd9365_8->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    switch (panel_dev_config->bits_per_pixel)
    {
    case 16: // RGB565
        jd9365_8->colmod_val = 0x55;
        break;
    case 18: // RGB666
        jd9365_8->colmod_val = 0x66;
        break;
    case 24: // RGB888
        jd9365_8->colmod_val = 0x77;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    jd9365_8->io = io;
    jd9365_8->init_cmds = vendor_config->init_cmds;
    jd9365_8->init_cmds_size = vendor_config->init_cmds_size;
    jd9365_8->lane_num = vendor_config->mipi_config.lane_num;
    jd9365_8->reset_gpio_num = panel_dev_config->reset_gpio_num;
    jd9365_8->flags.reset_level = panel_dev_config->flags.reset_active_high;

    // Create MIPI DPI panel
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus, vendor_config->mipi_config.dpi_config, &panel_handle), err, TAG,
                      "create MIPI DPI panel failed");
    ESP_LOGD(TAG, "new MIPI DPI panel @%p", panel_handle);

    // Save the original functions of MIPI DPI panel
    jd9365_8->del = panel_handle->del;
    jd9365_8->init = panel_handle->init;
    // Overwrite the functions of MIPI DPI panel
    panel_handle->del = panel_jd9365_8_del;
    panel_handle->init = panel_jd9365_8_init;
    panel_handle->reset = panel_jd9365_8_reset;
    panel_handle->mirror = panel_jd9365_8_mirror;
    panel_handle->swap_xy = panel_jd9365_8_swap_xy;
    panel_handle->set_gap = panel_jd9365_8_set_gap;
    panel_handle->invert_color = panel_jd9365_8_invert_color;
    panel_handle->disp_on_off = panel_jd9365_8_disp_on_off;
    panel_handle->user_data = jd9365_8;
    *ret_panel = panel_handle;
    ESP_LOGD(TAG, "new jd9365_8 panel @%p", jd9365_8);

    return ESP_OK;

err:
    if (jd9365_8)
    {
        if (panel_dev_config->reset_gpio_num >= 0)
        {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(jd9365_8);
    }
    return ret;
}

static const jd9365_8_lcd_init_cmd_t vendor_specific_init_default[] = {
    // {cmd, { data }, data_size, delay_ms}
    {0x11, (uint8_t[]){0x00}, 1, 500},
    {0xE0, (uint8_t[]){0x00}, 1, 0},
    {0xE1, (uint8_t[]){0x93}, 1, 0},
    {0xE2, (uint8_t[]){0x65}, 1, 0},
    {0xE3, (uint8_t[]){0xF8}, 1, 0},
    {0x80, (uint8_t[]){0x01}, 1, 0},
#if JD9356_8_TEST_PATTERN
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x83, (uint8_t[]){0x04}, 1, 0},
    {0xE0, (uint8_t[]){0x01}, 1, 0},
    {0x4A, (uint8_t[]){0x35}, 1, 0},
#endif
    {0xE0, (uint8_t[]){0x00}, 1, 0},
    {0x29, (uint8_t[]){0x00}, 1, 50},
};

static esp_err_t panel_jd9365_8_del(esp_lcd_panel_t *panel)
{
    jd9365_8_panel_t *jd9365_8 = (jd9365_8_panel_t *)panel->user_data;

    if (jd9365_8->reset_gpio_num >= 0)
    {
        gpio_reset_pin(jd9365_8->reset_gpio_num);
    }
    // Delete MIPI DPI panel
    jd9365_8->del(panel);

    ESP_LOGD(TAG, "del jd9365_8 panel @%p", jd9365_8);
    free(jd9365_8);

    return ESP_OK;
}

static esp_err_t panel_jd9365_8_init(esp_lcd_panel_t *panel)
{
    jd9365_8_panel_t *jd9365_8 = (jd9365_8_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = jd9365_8->io;
    const jd9365_8_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    uint8_t lane_command = JD9365_8_DSI_2_LANE;
    bool is_user_set = true;
    bool is_cmd_overwritten = false;

    switch (jd9365_8->lane_num)
    {
    case 0:
        lane_command = JD9365_8_DSI_2_LANE;
        break;
    case 1:
        lane_command = JD9365_8_DSI_1_LANE;
        break;
    case 2:
        lane_command = JD9365_8_DSI_2_LANE;
        break;
    case 3:
        lane_command = JD9365_8_DSI_3_LANE;
        break;
    case 4:
        lane_command = JD9365_8_DSI_4_LANE;
        break;
    default:
        ESP_LOGE(TAG, "Invalid lane number %d", jd9365_8->lane_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ID[3];
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io, 0x04, ID, 3), TAG, "read ID failed");
    ESP_LOGI(TAG, "LCD ID: %02X %02X %02X", ID[0], ID[1], ID[2]);

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, JD9365_8_CMD_PAGE, (uint8_t[]){JD9365_8_PAGE_USER}, 1), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){
                                                                          jd9365_8->madctl_val,
                                                                      },
                                                  1),
                        TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]){
                                                                          jd9365_8->colmod_val,
                                                                      },
                                                  1),
                        TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, JD9365_8_CMD_DSI_INT0, (uint8_t[]){
                                                                               lane_command,
                                                                           },
                                                  1),
                        TAG, "send command failed");

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    if (jd9365_8->init_cmds)
    {
        init_cmds = jd9365_8->init_cmds;
        init_cmds_size = jd9365_8->init_cmds_size;
    }
    else
    {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(jd9365_8_lcd_init_cmd_t);
    }

    for (int i = 0; i < init_cmds_size; i++)
    {
        // Check if the command has been used or conflicts with the internal
        if (is_user_set && (init_cmds[i].data_bytes > 0))
        {
            switch (init_cmds[i].cmd)
            {
            case LCD_CMD_MADCTL:
                is_cmd_overwritten = true;
                jd9365_8->madctl_val = ((uint8_t *)init_cmds[i].data)[0];
                break;
            case LCD_CMD_COLMOD:
                is_cmd_overwritten = true;
                jd9365_8->colmod_val = ((uint8_t *)init_cmds[i].data)[0];
                break;
            default:
                is_cmd_overwritten = false;
                break;
            }

            if (is_cmd_overwritten)
            {
                is_cmd_overwritten = false;
                ESP_LOGW(TAG, "The %02Xh command has been used and will be overwritten by external initialization sequence",
                         init_cmds[i].cmd);
            }
        }

        // Send command
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));

        // Check if the current cmd is the "page set" cmd
        if ((init_cmds[i].cmd == JD9365_8_CMD_PAGE) && (init_cmds[i].data_bytes > 0))
        {
            is_user_set = (((uint8_t *)init_cmds[i].data)[0] == JD9365_8_PAGE_USER);
        }
    }
    ESP_LOGD(TAG, "send init commands success");

    ESP_RETURN_ON_ERROR(jd9365_8->init(panel), TAG, "init MIPI DPI panel failed");

    return ESP_OK;
}

static esp_err_t panel_jd9365_8_reset(esp_lcd_panel_t *panel)
{
    jd9365_8_panel_t *jd9365_8 = (jd9365_8_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = jd9365_8->io;

    // Perform hardware reset
    if (jd9365_8->reset_gpio_num >= 0)
    {
        gpio_set_level(jd9365_8->reset_gpio_num, !jd9365_8->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(jd9365_8->reset_gpio_num, jd9365_8->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(jd9365_8->reset_gpio_num, !jd9365_8->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    else if (io)
    { // Perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    return ESP_OK;
}

static esp_err_t panel_jd9365_8_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    jd9365_8_panel_t *jd9365_8 = (jd9365_8_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = jd9365_8->io;
    uint8_t command = 0;

    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_STATE, TAG, "invalid panel IO");

    if (invert_color_data)
    {
        command = LCD_CMD_INVON;
    }
    else
    {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_jd9365_8_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ESP_LOGE(TAG, "mirror is not supported by this panel");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_jd9365_8_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_LOGE(TAG, "swap_xy is not supported by this panel");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_jd9365_8_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ESP_LOGE(TAG, "set_gap is not supported by this panel");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_jd9365_8_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    jd9365_8_panel_t *jd9365_8 = (jd9365_8_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = jd9365_8->io;
    int command = 0;

    if (on_off)
    {
        command = LCD_CMD_DISPON;
    }
    else
    {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}
#endif
