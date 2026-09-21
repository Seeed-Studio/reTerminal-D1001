#pragma once

#include "lvgl.h"
#include "bsp_board_extra.h"
#include "esp_brookesia.hpp"

class TouchTest: public ESP_Brookesia_PhoneApp
{
public:
	TouchTest();
	~TouchTest();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;
    bool pause(void) override;
    bool resume(void) override;
    
private:
    static void position_change_cb(lv_event_t *e);
    static void rate_timer_cb(lv_timer_t *timer);

    lv_obj_t *rate_label_ = nullptr;
    lv_timer_t *rate_timer_ = nullptr;
    uint8_t rate_tick_ = 0;
};
