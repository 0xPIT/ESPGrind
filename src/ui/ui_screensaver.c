#include "../common.h"
#include "ui_screensaver.h"
#include "ui.h"
#include "bsp/waveshare-esp32s3-touch2.8.bsp.h"
#include "settings.h"

extern lv_obj_t **getAllButtons();
static lv_timer_t *screenSaverTimer = NULL;
static const uint32_t screenSaverTimeout = 1; // 10 minutes
static uint8_t normalBrightness = 0;
static bool screenSaverActive = false;

void retriggerScreenSaver(lv_event_t *e) {
    if (screenSaverActive) {
        _ui_screen_change(&ui_Main, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Main_screen_init);
        bsp_display_brightness_set(normalBrightness); // restore normal brightness
        screenSaverActive = false;
    }
    
    if (screenSaverTimer) {
        lv_timer_reset(screenSaverTimer);
        lv_timer_resume(screenSaverTimer);
    }
}

static void onscreenSaverTimer(lv_timer_t *timer) {
    if (!screenSaverActive) {
        _ui_screen_change(&ui_ScreenSaver, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenSaver_screen_init);
        settings_t *settings = settingsGet();
        normalBrightness = settings->brightness;
        bsp_display_brightness_set(normalBrightness / 20); // dim to 20% of normal brightness
        screenSaverActive = true;
    }
}

void setDefaultBrightness(uint32_t level) {
    normalBrightness = level;
}

void setupScreenSaver() {
    screenSaverTimer = lv_timer_create(onscreenSaverTimer, screenSaverTimeout * 60 * 1000, NULL);
    lv_timer_reset(screenSaverTimer);

    lv_obj_add_event_cb(ui_ScreenSaver, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_standbyLabel, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_standbyInfoLabel, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Main, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);

    lv_obj_t **allButtons = getAllButtons();
    for (int i = 0; i < allButtonsCount; i++) {
        lv_obj_add_event_cb(allButtons[i], retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    }
} 