//
// Espresso mill controller
// (c) 2025 karl@pitrich.com
//

#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int32_t timer_defaults[4];
    int32_t counters[4];
    int8_t last_focussed;
    int32_t brightness;
    int16_t screensaverTimeout;
} settings_t;

esp_err_t settingsLoad(void);
settings_t *settingsGet(void);
esp_err_t settingsSave(void);
void settingsSaveDeferred(void);
esp_err_t settingsInit(void);
