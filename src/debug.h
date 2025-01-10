//
// Espresso mill controller
// (c) 2025 karl@pitrich.com
//

#pragma once

#include "esp_timer.h"
#include "esp_log.h"
#include "stdio.h"
#include "stdint.h"
#include "lvgl.h"

#ifdef DEBUG
uint64_t last_event_print = 0;
void printEvent(lv_event_t *e) {
  struct {
    uint16_t event;
    const char *text;
  } event_to_text[] = {
      { LV_EVENT_PRESSED, "LV_EVENT_PRESSED" },
      { LV_EVENT_PRESSING, "LV_EVENT_PRESSING" },
      { LV_EVENT_PRESS_LOST, "LV_EVENT_PRESS_LOST" },
      { LV_EVENT_SHORT_CLICKED, "LV_EVENT_SHORT_CLICKED" },
      { LV_EVENT_LONG_PRESSED, "LV_EVENT_LONG_PRESSED" },
      { LV_EVENT_LONG_PRESSED_REPEAT, "LV_EVENT_LONG_PRESSED_REPEAT" },
      { LV_EVENT_CLICKED, "LV_EVENT_CLICKED" },
      { LV_EVENT_RELEASED, "LV_EVENT_RELEASED" },
      { LV_EVENT_SCROLL_BEGIN, "LV_EVENT_SCROLL_BEGIN" },
      { LV_EVENT_SCROLL_THROW_BEGIN, "LV_EVENT_SCROLL_THROW_BEGIN" },
      { LV_EVENT_SCROLL_END, "LV_EVENT_SCROLL_END" },
      { LV_EVENT_SCROLL, "LV_EVENT_SCROLL" },
      { LV_EVENT_GESTURE, "LV_EVENT_GESTURE" },
      { LV_EVENT_KEY, "LV_EVENT_KEY" },
      { LV_EVENT_ROTARY, "LV_EVENT_ROTARY" },
      { LV_EVENT_FOCUSED, "LV_EVENT_FOCUSED" },
      { LV_EVENT_DEFOCUSED, "LV_EVENT_DEFOCUSED" },
      { LV_EVENT_LEAVE, "LV_EVENT_LEAVE" },
      { LV_EVENT_HIT_TEST, "LV_EVENT_HIT_TEST" },
      { LV_EVENT_INDEV_RESET, "LV_EVENT_INDEV_RESET" },
      { LV_EVENT_HOVER_OVER, "LV_EVENT_HOVER_OVER" },
      { LV_EVENT_HOVER_LEAVE, "LV_EVENT_HOVER_LEAVE"} };

  uint64_t current_time = esp_timer_get_time();
  int code = 0;
  for (code = 0; code <= LV_EVENT_HOVER_LEAVE; code++) {
    if (lv_event_get_code(e) == event_to_text[code].event) {
      printf("\t%s (@%lld, ∆%lld)\n", event_to_text[code].text,
             current_time / 1000, (current_time - last_event_print) / 1000);
      break;
    }
  }

  if (code > LV_EVENT_HOVER_LEAVE) {
    printf("\tEvent %d (@%lld, ∆%lld)\n", lv_event_get_code(e),
           current_time / 1000, (current_time - last_event_print) / 1000);
  }

  last_event_print = current_time;
}


inline bool debounceTouchEvent() {
    static uint64_t last_touch_event_at = 0;
    const uint64_t debounce_time = 10 * 1000; // ms
    const uint64_t current_time = esp_timer_get_time();

    if (last_touch_event_at > 0 && current_time - last_touch_event_at < debounce_time) {
        printf("debounceTouchEvent: %lld\n", current_time - last_touch_event_at);
        return true;
    }
    
    last_touch_event_at = current_time;

    return false;
}
#endif
