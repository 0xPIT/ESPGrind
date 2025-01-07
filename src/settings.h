
#pragma once

#include <stdint.h>
#include "esp_log.h"
#include "esp_check.h"

typedef struct {
    int32_t timer_defaults[4];
    int32_t counters[4];
    int8_t last_focussed;
    int32_t brightness;
} settings_t;

esp_err_t settingsLoad(void);
settings_t *settingsGet(void);
esp_err_t settingsSave(void);
void settingsSaveDeferred(void);
