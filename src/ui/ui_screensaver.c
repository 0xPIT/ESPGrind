#include "../common.h"
#include "ui_screensaver.h"
#include "ui.h"
#include "bsp/waveshare-esp32s3-touch2.8.bsp.h"
#include "settings.h"
#include "esp_log.h"

static const char *TAG = "SSAVER";

extern lv_obj_t **getAllButtons();
static lv_timer_t *screenSaverTimer = NULL;
static bool screenSaverActive = false;
static const int16_t dimBrightness = 8; // dim down to some arbitrary 8%

void retriggerScreenSaver(lv_event_t *e) {
    if (screenSaverActive) {
        settings_t *settings = settingsGet();
        _ui_screen_change(&ui_Main, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Main_screen_init);
        bsp_display_brightness_set(settings->brightness); // restore normal brightness
        screenSaverActive = false;
    }
    
    if (screenSaverTimer) {
        lv_timer_reset(screenSaverTimer);
        lv_timer_resume(screenSaverTimer);
    }
}

static void onScreenSaverTimer(lv_timer_t *timer) {
    if (!screenSaverActive) {
        _ui_screen_change(&ui_ScreenSaver, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ScreenSaver_screen_init);
        bsp_display_brightness_set(dimBrightness);
        screenSaverActive = true;
        if (screenSaverTimer) {
           lv_timer_pause(screenSaverTimer);
        }
    }
}

void setupScreenSaver() {
    const uint16_t screensaverTimeoutsIndexToMinutes[5] = { 0, 5, 15, 30, 60 };
    settings_t *settings = settingsGet();

    if (settings->screensaverTimeout == 0) {
        ESP_LOGI(TAG, "screensaver timeout is 0, not setting up screen saver\n");
        if (screenSaverTimer) {
            lv_timer_pause(screenSaverTimer);
        }
        return;
    }

    lv_obj_add_event_cb(ui_ScreenSaver, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_standbyLabel, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_standbyInfoLabel, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Main, retriggerScreenSaver, LV_EVENT_CLICKED, NULL);

    lv_obj_t **allButtons = getAllButtons();
    for (int i = 0; i < allButtonsCount; i++) {
        lv_obj_add_event_cb(allButtons[i], retriggerScreenSaver, LV_EVENT_CLICKED, NULL);
    }

    ESP_LOGI(TAG, "setting up screen saver timer for %d minutes\n", screensaverTimeoutsIndexToMinutes[settings->screensaverTimeout]);
    if (screenSaverTimer == NULL) {
        screenSaverTimer = lv_timer_create(onScreenSaverTimer, screensaverTimeoutsIndexToMinutes[settings->screensaverTimeout] * 60 * 1000, NULL);
    }
    lv_timer_reset(screenSaverTimer);
    lv_timer_resume(screenSaverTimer);
}
