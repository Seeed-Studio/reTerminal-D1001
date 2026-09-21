#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp-bsp.h"
#include "touch.h"
#include "TouchTest.hpp"

using namespace std;

#define scr_act_width() lv_obj_get_width(lv_scr_act())
#define scr_act_height() lv_obj_get_height(lv_scr_act())

static const char *TAG = "TouchTest";

LV_IMG_DECLARE(img_app_touch);

static lv_obj_t *canvas_disp;
static lv_color_t *canvas_buf;

TouchTest::TouchTest():
    ESP_Brookesia_PhoneApp("Draw Dot", &img_app_touch, true, false, false)
{
    
}

TouchTest::~TouchTest()
{

}

bool TouchTest::run(void)
{
    lv_obj_add_event_cb(lv_scr_act(), position_change_cb, LV_EVENT_ALL, this);

    ESP_LOGI(TAG, "scr_act_width: %d, scr_act_height: %d", scr_act_width(), scr_act_height());

    canvas_buf = (lv_color_t *)heap_caps_malloc(scr_act_width() * scr_act_height() * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if(canvas_buf)
    {
        memset(canvas_buf, 0xff, scr_act_width() * scr_act_height() * sizeof(lv_color_t));

        canvas_disp = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas_disp, canvas_buf, scr_act_width(), scr_act_height(), LV_IMG_CF_TRUE_COLOR);
        lv_obj_center(canvas_disp);
    }
    else
    {
        ESP_LOGI(TAG, "canvas malloc fail");
    }

    /* Touch INT report-rate measurement (pure GPIO observation, does not
     * affect the LVGL touch read path). */
    esp_err_t err = bsp_touch_int_rate_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Start touch INT rate measurement failed: 0x%x", err);
    }

    /* Created after the canvas so the overlay stays on top. */
    rate_label_ = lv_label_create(lv_scr_act());
    lv_obj_set_pos(rate_label_, 10, 10);
    lv_obj_set_style_bg_color(rate_label_, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(rate_label_, LV_OPA_70, 0);
    lv_obj_set_style_text_color(rate_label_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_pad_all(rate_label_, 8, 0);
    lv_obj_clear_flag(rate_label_, LV_OBJ_FLAG_CLICKABLE);
    lv_label_set_text(rate_label_, "INT: -- Hz\n(no edges)");

    rate_tick_ = 0;
    rate_timer_ = lv_timer_create(rate_timer_cb, 500, this);

    return true;
}

bool TouchTest::back(void)
{
    notifyCoreClosed();
    return true;
}

bool TouchTest::close(void)
{
    /* Delete the timer first: its callback uses the label. */
    if (rate_timer_)
    {
        lv_timer_del(rate_timer_);
        rate_timer_ = nullptr;
    }
    if (rate_label_)
    {
        lv_obj_del(rate_label_);
        rate_label_ = nullptr;
    }

    if(canvas_buf)
    {
        free(canvas_buf);
    }
    return true;
}

bool TouchTest::init(void)
{
    return true;
}

bool TouchTest::pause(void)
{
    return true;
}

bool TouchTest::resume(void)
{
    return true;
}

static void draw_dot(lv_coord_t x, lv_coord_t y)
{
    const int r = 10;
    const int w = scr_act_width();
    const int h = scr_act_height();
    const lv_color_t color = lv_color_make(0xff, 0x00, 0x00);

    if (canvas_buf == NULL || w <= 0 || h <= 0)
    {
        return;
    }

    for (int dy = -r; dy <= r; dy++)
    {
        const int py = y + dy;
        if (py < 0 || py >= h)
        {
            continue;
        }
        for (int dx = -r; dx <= r; dx++)
        {
            const int px = x + dx;
            if (px < 0 || px >= w)
            {
                continue;
            }
            if (dx * dx + dy * dy > r * r)
            {
                continue;
            }
            canvas_buf[py * w + px] = color;
        }
    }

    lv_area_t area = { (lv_coord_t)(x - r), (lv_coord_t)(y - r),
                       (lv_coord_t)(x + r), (lv_coord_t)(y + r) };
    lv_obj_invalidate_area(canvas_disp, &area);
}

void TouchTest::position_change_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING)
    {
        lv_point_t p;
        lv_indev_t *indev = lv_indev_get_act();
        lv_indev_get_point(indev, &p);
        // ESP_LOGI(TAG, "Touch X: %d, Y: %d", p.x, p.y);

        static lv_coord_t last_x = -1, last_y = -1;
        if (code == LV_EVENT_PRESSING && p.x == last_x && p.y == last_y)
        {
            return;
        }
        last_x = p.x;
        last_y = p.y;

        draw_dot(p.x, p.y);
    }
}

void TouchTest::rate_timer_cb(lv_timer_t *timer)
{
    TouchTest *self = (TouchTest *)timer->user_data;
    bsp_touch_int_rate_info_t info = {0};

    bsp_touch_int_rate_get_info(&info);

    if (self->rate_label_)
    {
        if (info.falling_hz > 0)
        {
            lv_label_set_text_fmt(self->rate_label_,
                                  "INT: %u Hz\nmin %u / max %u ms",
                                  (unsigned)info.falling_hz,
                                  (unsigned)info.min_interval_ms,
                                  (unsigned)info.max_interval_ms);
        }
        else
        {
            lv_label_set_text(self->rate_label_, "INT: -- Hz\n(no edges)");
        }
    }

    /* Log once per second (timer runs at 500 ms). */
    self->rate_tick_++;
    if ((self->rate_tick_ % 2) == 0)
    {
        ESP_LOGI(TAG, "touch INT rate: %u Hz (min %u ms, max %u ms, rising %u Hz)",
                 (unsigned)info.falling_hz,
                 (unsigned)info.min_interval_ms,
                 (unsigned)info.max_interval_ms,
                 (unsigned)info.rising_hz);
    }
}
