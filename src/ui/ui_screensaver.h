#pragma once

#include "lvgl.h"

void setupScreenSaver(void);
void setDefaultBrightness(uint32_t level);
void retriggerScreensaver(lv_event_t *e);
