/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp-bsp.h"
#include "bsp_board_extra.h"

static const char *TAG = "bsp_extra_board";

static esp_codec_dev_handle_t play_dev_handle;
static esp_codec_dev_handle_t record_dev_handle;

static bool _is_audio_init = false;
static bool _is_player_init = false;
static int _vloume_intensity = CODEC_DEFAULT_DAC_VOLUME;

static audio_player_cb_t audio_idle_callback = NULL;
static void *audio_idle_cb_user_data = NULL;
static char audio_file_path[128];

/**************************************************************************************************
 *
 * Extra Board Function
 *
 **************************************************************************************************/

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting)
{
    // Volume saved when muting and restored when unmuting. Restoring volume is necessary
    // as es8311_set_voice_mute(true) results in voice volume (REG32) being set to zero.

    bsp_extra_codec_mute_set(setting == AUDIO_PLAYER_MUTE ? true : false);

    // restore the voice volume upon unmuting
    if (setting == AUDIO_PLAYER_UNMUTE) {
        ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(play_dev_handle, _vloume_intensity), TAG, "Set Codec volume failed");
    }

    return ESP_OK;
}

static void audio_callback(audio_player_cb_ctx_t *ctx)
{
    if (audio_idle_callback) {
        ctx->user_ctx = audio_idle_cb_user_data;
        audio_idle_callback(ctx);
    }
}

esp_err_t bsp_extra_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_read(record_dev_handle, audio_buffer, len);
    *bytes_read = len;
    return ret;
}

esp_err_t bsp_extra_i2s_write(const void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_write(play_dev_handle, (void *)audio_buffer, len);
    *bytes_written = len;
    return ret;
}

esp_err_t bsp_extra_codec_play_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    if (play_dev_handle) {
        ret = esp_codec_dev_close(play_dev_handle);
    }

    if (play_dev_handle) {
        ret |= esp_codec_dev_open(play_dev_handle, &fs);
    }

    return ret;
}

esp_err_t bsp_extra_codec_record_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    if (record_dev_handle) {
        ret |= esp_codec_dev_close(record_dev_handle);
    }

    if (record_dev_handle) {
        ret |= esp_codec_dev_open(record_dev_handle, &fs);
    }

    if (record_dev_handle) {
        ret |= esp_codec_dev_set_in_channel_gain(record_dev_handle, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), CODEC_DEFAULT_ADC_VOLUME);
        ret |= esp_codec_dev_set_in_channel_gain(record_dev_handle, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1), CODEC_DEFAULT_ADC_VOLUME);
        ret |= esp_codec_dev_set_in_channel_gain(record_dev_handle, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(2), 0);
        ret |= esp_codec_dev_set_in_channel_gain(record_dev_handle, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(3), 0);
    }

    return ret;
}

esp_err_t bsp_extra_codec_volume_set(int volume, int *volume_set)
{
    if (volume > 93) volume = 93;

    ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(play_dev_handle, volume), TAG, "Set Codec volume failed");
    _vloume_intensity = volume;

    ESP_LOGI(TAG, "Setting volume: %d", volume);

    return ESP_OK;
}

int bsp_extra_codec_volume_get(void)
{
    return _vloume_intensity;
}

esp_err_t bsp_extra_codec_mute_set(bool enable)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_set_out_mute(play_dev_handle, enable);
    return ret;
}

esp_err_t bsp_extra_codec_dev_stop(void)
{
    esp_err_t ret = ESP_OK;

    if (play_dev_handle) {
        ret = esp_codec_dev_close(play_dev_handle);
    }

    if (record_dev_handle) {
        ret = esp_codec_dev_close(record_dev_handle);
    }
    return ret;
}

esp_err_t bsp_extra_codec_dev_resume(void)
{
    esp_err_t ret = ESP_OK;
    ret = bsp_extra_codec_play_set_fs(CODEC_DEFAULT_DAC_SAMPLE_RATE, CODEC_DEFAULT_DAC_BIT_WIDTH, CODEC_DEFAULT_DAC_CHANNEL);
    ret |= bsp_extra_codec_record_set_fs(CODEC_DEFAULT_ADC_SAMPLE_RATE, CODEC_DEFAULT_ADC_BIT_WIDTH, CODEC_DEFAULT_ADC_CHANNEL);
    return ret;
}

esp_err_t bsp_extra_codec_init()
{
    if (_is_audio_init) {
        return ESP_OK;
    }

    play_dev_handle = bsp_audio_codec_speaker_init();
    assert((play_dev_handle) && "play_dev_handle not initialized");
    bsp_extra_codec_play_set_fs(CODEC_DEFAULT_DAC_SAMPLE_RATE, CODEC_DEFAULT_DAC_BIT_WIDTH, CODEC_DEFAULT_DAC_CHANNEL);

    record_dev_handle = bsp_audio_codec_microphone_init();
    assert((record_dev_handle) && "record_dev_handle not initialized");
    bsp_extra_codec_record_set_fs(CODEC_DEFAULT_ADC_SAMPLE_RATE, CODEC_DEFAULT_ADC_BIT_WIDTH, CODEC_DEFAULT_ADC_CHANNEL);

    _is_audio_init = true;

    ESP_LOGI(TAG, "codec init ok");

    return ESP_OK;
}

esp_err_t bsp_extra_player_init(void)
{
    if (_is_player_init) {
        return ESP_OK;
    }

    audio_player_config_t config = { .mute_fn = audio_mute_function,
                                     .write_fn = bsp_extra_i2s_write,
                                     .clk_set_fn = bsp_extra_codec_play_set_fs,
                                     .priority = 5
                                   };
    ESP_RETURN_ON_ERROR(audio_player_new(config), TAG, "audio_player_init failed");
    audio_player_callback_register(audio_callback, NULL);

    _is_player_init = true;

    ESP_LOGI(TAG, "codec player init ok");

    return ESP_OK;
}

esp_err_t bsp_extra_player_del(void)
{
    _is_player_init = false;

    ESP_RETURN_ON_ERROR(audio_player_delete(), TAG, "audio_player_delete failed");

    return ESP_OK;
}

esp_err_t bsp_extra_file_instance_init(const char *path, file_iterator_instance_t **ret_instance)
{
    ESP_RETURN_ON_FALSE(path, ESP_FAIL, TAG, "path is NULL");
    ESP_RETURN_ON_FALSE(ret_instance, ESP_FAIL, TAG, "ret_instance is NULL");

    file_iterator_instance_t *file_iterator = file_iterator_new(path);
    ESP_RETURN_ON_FALSE(file_iterator, ESP_FAIL, TAG, "file_iterator_new failed, %s", path);

    *ret_instance = file_iterator;

    return ESP_OK;
}

esp_err_t bsp_extra_player_play_index(file_iterator_instance_t *instance, int index)
{
    ESP_RETURN_ON_FALSE(instance, ESP_FAIL, TAG, "instance is NULL");

    ESP_LOGI(TAG, "play_index(%d)", index);
    char filename[128];
    int retval = file_iterator_get_full_path_from_index(instance, index, filename, sizeof(filename));
    ESP_RETURN_ON_FALSE(retval != 0, ESP_FAIL, TAG, "file_iterator_get_full_path_from_index failed");

    ESP_LOGI(TAG, "opening file '%s'", filename);
    FILE *fp = fopen(filename, "rb");
    ESP_RETURN_ON_FALSE(fp, ESP_FAIL, TAG, "unable to open file");

    ESP_LOGI(TAG, "Playing '%s'", filename);
    ESP_RETURN_ON_ERROR(audio_player_play(fp), TAG, "audio_player_play failed");

    memcpy(audio_file_path, filename, sizeof(audio_file_path));

    return ESP_OK;
}

esp_err_t bsp_extra_player_play_file(const char *file_path)
{
    ESP_LOGI(TAG, "opening file '%s'", file_path);
    FILE *fp = fopen(file_path, "rb");
    ESP_RETURN_ON_FALSE(fp, ESP_FAIL, TAG, "unable to open file");

    ESP_LOGI(TAG, "Playing '%s'", file_path);
    ESP_RETURN_ON_ERROR(audio_player_play(fp), TAG, "audio_player_play failed");

    memcpy(audio_file_path, file_path, sizeof(audio_file_path));

    return ESP_OK;
}

void bsp_extra_player_register_callback(audio_player_cb_t cb, void *user_data)
{
    audio_idle_callback = cb;
    audio_idle_cb_user_data = user_data;
}

bool bsp_extra_player_is_playing_by_path(const char *file_path)
{
    return (strcmp(audio_file_path, file_path) == 0);
}

bool bsp_extra_player_is_playing_by_index(file_iterator_instance_t *instance, int index)
{
    return (index == file_iterator_get_index(instance));
}

esp_err_t bsp_extra_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    esp_err_t ret = ESP_OK;
    int audio_chunksize = buffer_len / (sizeof(int16_t) * CODEC_DEFAULT_ADC_CHANNEL);
    int16_t ref1 = 0, ref2 = 0, ref3 = 0, ref4 = 0;
    ret = esp_codec_dev_read(record_dev_handle, (void *)buffer, buffer_len);
    if (!is_get_raw_channel) {
        for (int i = 0; i < audio_chunksize; i++) {
            ref1 = buffer[4 * i + 0];
            ref3 = buffer[4 * i + 1];
            ref2 = buffer[4 * i + 2];
            ref4 = buffer[4 * i + 3];
            buffer[4 * i + 0 ] = ref1;
            buffer[4 * i + 1 ] = ref2;
            buffer[4 * i + 2 ] = ref3;
            buffer[4 * i + 3 ] = ref4;
        }
    }
    return ret;
}

int bsp_extra_get_feed_channel(void)
{
    return CODEC_DEFAULT_ADC_CHANNEL;
}

char *bsp_extra_get_input_format(void)
{
    return "MMRN";
}
