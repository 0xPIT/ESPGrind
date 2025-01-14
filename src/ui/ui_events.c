//
// Espresso mill controller
// (c) 2025 karl@pitrich.com
//
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "bsp/waveshare-esp32s3-touch2.8.bsp.h"

#include "mill.h"
#include "settings.h"
#include "ui.h"
#include "../common.h"
#include "ui_screensaver.h"

static const char *TAG = "UIE";

static bool initialized = false;

static lv_timer_t *timer = NULL;
static const uint8_t timerStep = 20; // ms
static const uint32_t maxManualGrindTime = 6000; // 60.00 seconds in display units
static uint32_t timerStartValue = 0;

static lv_obj_t *lastFocussed = NULL;
lv_group_t *focusGroup = NULL;
bool focusChange = false;

static lv_obj_t **getGrindButtons() {
    static lv_obj_t *grindButtons[grindButtonsCount];
    grindButtons[0] = ui_GrindI;
    grindButtons[1] = ui_GrindII;
    grindButtons[2] = ui_GrindIII;
    grindButtons[3] = ui_GrindM;
    return grindButtons;
}

lv_obj_t **getAllButtons() {
    static lv_obj_t *allButtons[allButtonsCount];
    allButtons[0] = ui_GrindI;
    allButtons[1] = ui_GrindII;
    allButtons[2] = ui_GrindIII;
    allButtons[3] = ui_GrindM;
    allButtons[4] = ui_Plus;
    allButtons[5] = ui_Minus;
    allButtons[6] = ui_Timer;
    allButtons[7] = ui_Settings;
    return allButtons;
}

void saveChangedTime() {
    settings_t *settings = settingsGet();
    if (settings->last_focussed != -1) {
        for (int i = 0; i < 3; i++) {
            if (i == settings->last_focussed) {
                settings->timer_defaults[i] = lv_spinbox_get_value(ui_Timer);
                settingsSaveDeferred();
            }
        }
    }
}

void countButtonEvent(lv_obj_t *button) {
    settings_t *settings = settingsGet();
    lv_obj_t **grindButtons = getGrindButtons();
    for (int i = 0; i < grindButtonsCount; i++) {
        if (grindButtons[i] == button) {
            settings->counters[i]++;
            settingsSaveDeferred();
            break;
        }
    }
}

static void disableClickOnOtherThan(lv_obj_t *button) {
    lv_obj_t **allButtons = getAllButtons();
    for (int i = 0; i < allButtonsCount; i++) {
        if (allButtons[i] == button) {
            lv_obj_add_flag(allButtons[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_remove_state(allButtons[i], LV_STATE_DISABLED);
        }
        else {
            lv_obj_remove_flag(allButtons[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_state(allButtons[i], LV_STATE_DISABLED, true);
        }
    }
}

static void enableClickOnAll() {
    lv_obj_t **allButtons = getAllButtons();
    for (int i = 0; i < allButtonsCount; i++) {
        lv_obj_add_flag(allButtons[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_state(allButtons[i], LV_STATE_DISABLED);
    }
}

void onDecreaseTime1(lv_event_t *e) {
    lv_spinbox_decrement(ui_Timer);
    saveChangedTime();
}

void onIncreaseTime1(lv_event_t *e) {
    lv_spinbox_increment(ui_Timer);
    saveChangedTime();
}

static void updateTimerDisplay(int32_t value) {
    lv_spinbox_set_value(ui_Timer, value);
}

static bool timerRunning() {
    return timer && lv_timer_get_paused(timer) == false;
}

static void handleManualGrindTimer(void) {
    int32_t currentValue = lv_spinbox_get_value(ui_Timer);
    int32_t newValue = currentValue + (timerStep / 10); // count up

    if (newValue >= maxManualGrindTime) {
        millOff();
        lv_timer_pause(timer);
        newValue = maxManualGrindTime;
        enableClickOnAll();
    }

    updateTimerDisplay(newValue);
}

static void stopPresetGrinding(lv_obj_t *target) {
    lv_timer_pause(timer);
    millOff();
    updateTimerDisplay(timerStartValue);
    lv_arc_set_value(target, 100);
    enableClickOnAll();
}

static void handlePresetGrindTimer(lv_obj_t *target) {
    int32_t currentValue = lv_spinbox_get_value(ui_Timer);
    int32_t newValue = currentValue - (timerStep / 10); // count down

    if (newValue < 0) {
        stopPresetGrinding(target);
        return;
    }

    updateTimerDisplay(newValue);
    float arcValue = 100.0 / (float)timerStartValue * (float)newValue;
    lv_arc_set_value(target, (int32_t)arcValue);
}

static void onTimer(lv_timer_t *timer) {
    lv_obj_t *target = lv_timer_get_user_data(timer);
    
    if (target == ui_GrindM) {
        handleManualGrindTimer();
    }
    else {
        handlePresetGrindTimer(target);
    }
}

void onGrindFocussed(lv_event_t *e) {
    if (timerRunning()) return;

    lv_obj_t *ui_Element = lv_event_get_current_target(e);
    lv_obj_t **grindButtons = getGrindButtons();
    settings_t *settings = settingsGet();

    for (int i = 0; i < grindButtonsCount; i++) {
        if (grindButtons[i] == ui_Element) {
            updateTimerDisplay(settings->timer_defaults[i]);

            if (lastFocussed != ui_Element) {
                focusChange = true;
            }
            lastFocussed = ui_Element;
   
            if (initialized) {
                if (settings->last_focussed != i) {
                    settings->last_focussed = i;
                    settingsSaveDeferred();
                }
            }

            break;
        }
    }
}

static void ensureTimer() {
    if (!timer) {
        timer = lv_timer_create(onTimer, timerStep, NULL);
        lv_timer_set_period(timer, timerStep);
        lv_timer_pause(timer);
    }
}

static void startMillFromButton(lv_obj_t *button) {
    if (!timer) {
        ESP_LOGE(TAG, " timer not initialized");
        return;
    }

    disableClickOnOtherThan(button);
    lv_timer_reset(timer);
    lv_timer_resume(timer);
    millOn();
    countButtonEvent(button);
}

void onManualGrindPush(lv_event_t *e) {
    lv_obj_t *clickedGrindButton = lv_event_get_current_target(e);
    lv_event_code_t currentEvent = lv_event_get_code(e);

    if (lastFocussed != clickedGrindButton) {
        focusChange = false;
        return;
    }

    ensureTimer();

    if (currentEvent == LV_EVENT_RELEASED) {
        if (timerRunning()) {
            millOff();
            lv_timer_pause(timer);
            enableClickOnAll();
        }
        return;
    }

    if (currentEvent == LV_EVENT_PRESSED && !timerRunning()) {
        updateTimerDisplay(0); // M starts at 0 every time
        lv_arc_set_value(clickedGrindButton, 100);
        lv_timer_set_user_data(timer, lv_event_get_current_target(e));
        timerStartValue = 0;
        lv_timer_set_repeat_count(timer, maxManualGrindTime * (timerStep / 10)); 
        startMillFromButton(clickedGrindButton);
    }
}

void onGrindClicked(lv_event_t *e) {
    lv_obj_t *clickedGrindButton = lv_event_get_current_target(e);

    bool isManualGrind = clickedGrindButton == ui_GrindM;
    if (isManualGrind) {
        // click event seems reqired to allow focus, but event ignored.
        return;
    }

    if (timerRunning()) {
        stopPresetGrinding(clickedGrindButton);
        return;
    }

    if (focusChange) {
        focusChange = false;
        return;
    }

    ensureTimer();

    lv_arc_set_value(clickedGrindButton, 100);
    lv_timer_set_user_data(timer, clickedGrindButton);
    timerStartValue = lv_spinbox_get_value(ui_Timer);
    lv_timer_set_repeat_count(timer, timerStartValue * (timerStep / 10));

    startMillFromButton(clickedGrindButton);
}

void editModeEnable() {
    lv_obj_remove_flag(ui_Plus, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(ui_Minus, LV_OBJ_FLAG_HIDDEN);
    lv_spinbox_set_cursor_pos(ui_Timer, 1); // first decimal
}

void editModeDisable() {
    lv_obj_add_flag(ui_Plus, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Minus, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_state(ui_Timer, LV_STATE_FOCUSED);
}

void editEnableChanged(lv_event_t *e) {
    if (lv_obj_has_state(ui_EditEnable, LV_STATE_CHECKED)) {
        editModeEnable();
    } 
    else {
        editModeDisable();
    }
}

void onBrightnessChanged(lv_event_t *e) {
    settings_t *settings = settingsGet();
    settings->brightness = lv_slider_get_value(ui_BrightnessSlider);
    bsp_display_brightness_set(settings->brightness);
    settingsSaveDeferred();
}

void onSettingsScreenLoaded(lv_event_t *e) {
    // update gui with elements not available in squareline studio
    lv_label_set_text(ui_SettingsExitLabel, LV_SYMBOL_LEFT);
    lv_label_set_text(ui_VersionLabel, GIT_VERSION);

    settings_t *settings = settingsGet();
    lv_slider_set_value(ui_BrightnessSlider, settings->brightness, LV_ANIM_OFF);
    lv_roller_set_selected(ui_ScreensaverTimeout, settings->screensaverTimeout, LV_ANIM_OFF);

    lv_obj_t *grindCounters[4] = {ui_Counter1, ui_Counter2, ui_Counter3, ui_Counter4};
    for (int i = 0; i < 4; i++) {
        lv_label_set_text_fmt(grindCounters[i], "%ld", settings->counters[i]);
    }
}

void onMainScreenLoaded(lv_event_t *e) {
    // update gui with elements not available in squareline studio
    lv_label_set_text(ui_SettingsLabel, LV_SYMBOL_SETTINGS);
    editEnableChanged(NULL);
}

void createButtonFocusGroup() {
    focusGroup = lv_group_create();
    lv_obj_t **grindButtons = getGrindButtons();
    for (int i = 0; i < grindButtonsCount; i++) {
        lv_group_add_obj(focusGroup, grindButtons[i]);
    }
}

void onScreensaverTimeoutchanged(lv_event_t *e) {
    uint16_t timeoutIdx = lv_roller_get_selected(ui_ScreensaverTimeout);
    if (timeoutIdx < 5) {
        settings_t *settings = settingsGet();
        settings->screensaverTimeout = timeoutIdx;
        settingsSaveDeferred();
        setupScreenSaver();
    }
}

void ui_InitialActions(lv_event_t *e) {
    millInit();
    settingsInit();
    editModeDisable();
    setupScreenSaver();

    lv_obj_add_event_cb(ui_ScreensaverTimeout, onScreensaverTimeoutchanged, LV_EVENT_VALUE_CHANGED, NULL);

    createButtonFocusGroup();

    lv_obj_t **grindButtons = getGrindButtons();
    settings_t *settings = settingsGet();

    bsp_display_brightness_set(settings->brightness);

    for (int i = 0; i < grindButtonsCount; i++) {
        if (i == settings->last_focussed) {
            lastFocussed = grindButtons[settings->last_focussed];
            lv_group_focus_obj(lastFocussed);
            focusChange = false;
            break;
        }
    }

    ESP_LOGI(TAG, "ESPGrind git version: %s", GIT_VERSION);

    initialized = true;
}
