//
// Espresso mill controller
// (c)2025 karl@pitrich.com
//
// Bugs:
// 1. the manual grund button does not behave corectly: 
//    1a. hold mode: it seems to depend on where you put your finger, probably some even proparation issue?
//    2b. toggle mode: when the timer runs, the next click to turn it off never arrives (no events arrive)
//
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "bsp/esp-box.h"

#include "mill.h"
#include "settings.h"
#include "ui.h"

static bool initialized = false;
static uint32_t previousValue = 0;

static lv_timer_t *timer = NULL;
static const uint8_t timerStep = 20; // ms
static const uint32_t maxManualGrindTime = 6000; // 60.00 seconds in display units

static lv_obj_t *lastFocussed = NULL;
lv_group_t *focusGroup = NULL;
bool focusChange = false;

static lv_obj_t **getGrindButtons() {
    static lv_obj_t *grindButtons[4];
    grindButtons[0] = ui_GrindI;
    grindButtons[1] = ui_GrindII;
    grindButtons[2] = ui_GrindIII;
    grindButtons[3] = ui_GrindM;
    return grindButtons;
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

static void handleManualGrind(void) {
    int32_t current_value = lv_spinbox_get_value(ui_Timer);
    int32_t new_value = current_value + (timerStep / 10); // count up

    if (new_value >= maxManualGrindTime) {
        millOff();
        lv_timer_pause(timer);
        new_value = maxManualGrindTime;
    }

    updateTimerDisplay(new_value);
}

static void handlePresetGrind(lv_obj_t *target) {
    int32_t current_value = lv_spinbox_get_value(ui_Timer);
    int32_t new_value = current_value - (timerStep / 10); // count down

    if (new_value < 0) {
        lv_timer_pause(timer);
        millOff();
        updateTimerDisplay(previousValue);
        lv_arc_set_value(target, 100);
        return;
    }

    updateTimerDisplay(new_value);

    float arc_value = 100.0 / (float)previousValue * ((float)new_value);
    lv_arc_set_value(target, (int32_t)arc_value);
}

static void onTimer(lv_timer_t *timer) {
    lv_obj_t *target = lv_timer_get_user_data(timer);
    
    if (target == ui_GrindM) {
        handleManualGrind();
    }
    else {
        handlePresetGrind(target);
    }
}

void onGrindFocussed(lv_event_t *e) {
    if (timerRunning()) return;

    lv_obj_t **grindButtons = getGrindButtons();
    lv_obj_t *ui_Element = lv_event_get_current_target(e);
    settings_t *settings = settingsGet();

    for (int i = 0; i < 4; i++) {
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
        }
    }
}

void countButtonEvent(lv_obj_t *button) {
    settings_t *settings = settingsGet();
    lv_obj_t **grindButtons = getGrindButtons();
    for (int i = 0; i < 4; i++) {
        if (grindButtons[i] == button) {
            settings->counters[i]++;
            settingsSaveDeferred();
            break;
        }
    }
}

void onGrindClicked(lv_event_t *e) {
    lv_obj_t *ui_Element = lv_event_get_current_target(e);

    if (timerRunning()) {
        if (millIsMilling() && ui_Element == ui_GrindM) {
            millOff();
            lv_timer_pause(timer);
        }
        return;
    }

    if (focusChange) {
        focusChange = false;
        return;
    }

    if (!timer) {
        timer = lv_timer_create(onTimer, timerStep, NULL);
        lv_timer_pause(timer);
    }

    lv_arc_set_value(ui_Element, 100);
    lv_timer_set_user_data(timer, ui_Element);
    lv_timer_set_period(timer, timerStep);

    if (ui_Element == ui_GrindM) {
        previousValue = 0;
        lv_timer_set_repeat_count(timer, maxManualGrindTime * (timerStep / 10));
    } else {
        previousValue = lv_spinbox_get_value(ui_Timer);
        lv_timer_set_repeat_count(timer, previousValue * (timerStep / 10));
    }

    lv_timer_reset(timer);
    lv_timer_resume(timer);
    millOn();
    countButtonEvent(ui_Element);
}

void editModeEnable() {
    lv_obj_remove_flag(ui_Plus, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(ui_Minus, LV_OBJ_FLAG_HIDDEN);
    lv_spinbox_set_cursor_pos(ui_Timer, 1);
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

void createFocusGroup() {
    focusGroup = lv_group_create();
    lv_obj_t **grindButtons = getGrindButtons();
    for (int i = 0; i < 4; i++) {
        lv_group_add_obj(focusGroup, grindButtons[i]);
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

    settings_t *settings = settingsGet();
    lv_slider_set_value(ui_BrightnessSlider, settings->brightness, LV_ANIM_OFF);

    lv_obj_t *grindCounters[4] = {ui_Counter1, ui_Counter2, ui_Counter3, ui_Counter4};
    for (int i = 0; i < 4; i++) {
        lv_label_set_text_fmt(grindCounters[i], "%ld", settings->counters[i]);
    }
}

void onMainScreenLoaded(lv_event_t *e) {
    editEnableChanged(NULL);
}

void ui_InitialActions(lv_event_t *e) {
    millInit();
    editModeDisable();
    createFocusGroup();

    lv_label_set_text(ui_SettingsLabel, LV_SYMBOL_SETTINGS);

    lv_obj_t **grindButtons = getGrindButtons();
    settings_t *settings = settingsGet();

    lv_slider_set_value(ui_BrightnessSlider, settings->brightness, LV_ANIM_OFF);
    bsp_display_brightness_set(settings->brightness);

    for (int i = 0; i < 4; i++) {
        if (i == settings->last_focussed) {
            lastFocussed = grindButtons[settings->last_focussed];
            lv_group_focus_obj(lastFocussed);
            focusChange = false;
        }
    }

    initialized = true;
}
