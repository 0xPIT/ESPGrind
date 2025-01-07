//
// Espresso mill controller
// (c) 2025 karl@pitrich.com
//

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "settings.h"

#define NAME_SPACE "sys_param"
#define KEY "param"
static const char *TAG = "Settings";

static SemaphoreHandle_t settingsMutex = NULL;
static SemaphoreHandle_t saveSemaphore = NULL;
static TaskHandle_t settingsTaskHandle = NULL;

static const uint32_t deferredSaveDelay = 5 * 1000 * 1000;  // 5s in µS
static esp_timer_handle_t deferredSaveTimer = NULL;

static settings_t currentSettings = { 0 };
static const settings_t defaultSettings = {
    // Button             I   II  III  M
    .timer_defaults = { 150, 260, 370, 0 },
    .counters =       {   0,   0,   0, 0 },

    .last_focussed = -1, // resume where left off
    .brightness = 50
};

static void settingsCheck(settings_t *param) {
    bool restore = false;

    for (uint8_t i = 0; i < 3; i++) {
        if (   param->timer_defaults[i] <= 0 
            || param->timer_defaults[i] > 6000
            || param->counters[i] < 0)
        {
            restore = true;
            break;
        }
    }

    if (param->brightness < 1 || param->brightness > 100) {
        restore = true;
    }

    if (restore) {
        ESP_LOGW(TAG, "restore defaults settings");
        memcpy(&currentSettings, &defaultSettings, sizeof(settings_t));
        settingsSave();
    }
}

static esp_err_t settingsSaveToNVS() {
    ESP_LOGI(TAG, "saving settings to NVS");
    settingsCheck(&currentSettings);
    
    nvs_handle_t my_handle = { 0 };
    esp_err_t err = nvs_open(NAME_SPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(my_handle, KEY, &currentSettings, sizeof(settings_t));
    err |= nvs_commit(my_handle);
    nvs_close(my_handle);
    
    return err;
}

static void settingsSaveTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(saveSemaphore, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(settingsMutex, portMAX_DELAY) == pdTRUE) {
                settingsSaveToNVS();
                xSemaphoreGive(settingsMutex);
            }
        }
    }
}

esp_err_t settingsLoad() {
    nvs_handle_t my_handle = 0;
    esp_err_t ret = nvs_open(NAME_SPACE, NVS_READONLY, &my_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "settings not found, set to default");
        memcpy(&currentSettings, &defaultSettings, sizeof(settings_t));
        settingsSave();
        return ESP_OK;
    }

    ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "nvs open failed (0x%x)", ret);

    size_t len = sizeof(settings_t);
    ret = nvs_get_blob(my_handle, KEY, &currentSettings, &len);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "can't read param");
    nvs_close(my_handle);

    settingsCheck(&currentSettings);
    return ret;

err:
    if (my_handle) {
        nvs_close(my_handle);
    }
    return ret;
}

settings_t *settingsGet() {
    settings_t *settings = NULL;
    if (xSemaphoreTake(settingsMutex, portMAX_DELAY) == pdTRUE) {
        settings = &currentSettings;
        xSemaphoreGive(settingsMutex);
    }
    return settings;
}

esp_err_t settingsSave() {
    xSemaphoreGive(saveSemaphore);
    return ESP_OK;
}

static void saveSettingsCallback(void* arg) {
    ESP_LOGI(TAG, "deferred saving settings");
    settingsSave();
}

void settingsSaveDeferred()  {
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

esp_err_t settingsInit() {
    #define SETTINGS_TASK_STACK_SIZE 4096
    #define SETTINGS_TASK_PRIORITY 2

    settingsMutex = xSemaphoreCreateMutex();
    if (settingsMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create settings mutex");
        return ESP_FAIL;
    }

    saveSemaphore = xSemaphoreCreateBinary();
    if (saveSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create save semaphore");
        vSemaphoreDelete(settingsMutex);
        return ESP_FAIL;
    }

    BaseType_t ret = xTaskCreate(
        settingsSaveTask,
        "settingsSaveTask",
        SETTINGS_TASK_STACK_SIZE,
        NULL,
        SETTINGS_TASK_PRIORITY,
        &settingsTaskHandle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create settings task");
        vSemaphoreDelete(settingsMutex);
        vSemaphoreDelete(saveSemaphore);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Settings task created");

    settingsLoad();

    return ESP_OK;
}
