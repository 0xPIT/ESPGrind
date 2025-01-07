#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "settings.h"

static const char *TAG = "Settings";
#define NAME_SPACE "sys_param"
#define KEY "param"

static const uint32_t deferredSaveDelay = 2 * 1000 * 1000;  // 2s in µS
static esp_timer_handle_t deferredSaveTimer = NULL;
static settings_t g_sys_param = {0};

static const settings_t g_default_sys_param = {
    .timer_defaults = { 150, 260, 370, 0 },
    .counters = { 0, 0, 0, 0 },
    .last_focussed = -1,
    .brightness = 50
};

static void settingsCheck(settings_t *param)
{
    for (uint8_t i = 0; i < 3; i++) {
        if (param->timer_defaults[i] <= 0 || param->timer_defaults[i] > 6000) {
            ESP_LOGW(TAG, "restore defaults settings");
            memcpy(&g_sys_param, &g_default_sys_param, sizeof(settings_t));
            return;
        }
    }

    if (param->brightness < 1 || param->brightness > 100) {
        ESP_LOGW(TAG, "restore defaults settings");
        memcpy(&g_sys_param, &g_default_sys_param, sizeof(settings_t));
        return;
    }
}

esp_err_t settingsLoad() 
{
    nvs_handle_t my_handle = 0;
    esp_err_t ret = nvs_open(NAME_SPACE, NVS_READONLY, &my_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "settings not found, set to default");
        memcpy(&g_sys_param, &g_default_sys_param, sizeof(settings_t));
        settingsSave();
        return ESP_OK;
    }

    ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "nvs open failed (0x%x)", ret);

    size_t len = sizeof(settings_t);
    ret = nvs_get_blob(my_handle, KEY, &g_sys_param, &len);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "can't read param");
    nvs_close(my_handle);

    settingsCheck(&g_sys_param);
    return ret;

err:
    if (my_handle) {
        nvs_close(my_handle);
    }
    return ret;
}

settings_t *settingsGet() 
{
    return &g_sys_param;
}

esp_err_t settingsSave() 
{
    ESP_LOGI(TAG, "saving settings");
    settingsCheck(&g_sys_param);
    nvs_handle_t my_handle = {0};
    esp_err_t err = nvs_open(NAME_SPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } 
    else {
        err = nvs_set_blob(my_handle, KEY, &g_sys_param, sizeof(settings_t));
        err |= nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    
    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

static void saveSettingsCallback(void* arg) 
{
    ESP_LOGI(TAG, "deferred saving settings");
    settingsSave();
}

void settingsSaveDeferred() 
{
    if (deferredSaveTimer == NULL) {
        const esp_timer_create_args_t timerArgs = {
            .callback = &saveSettingsCallback,
            .name = __func__,
            .skip_unhandled_events = true
        };
        esp_timer_create(&timerArgs, &deferredSaveTimer);
    }
    
    esp_timer_stop(deferredSaveTimer);
    esp_timer_start_once(deferredSaveTimer, deferredSaveDelay);
}
