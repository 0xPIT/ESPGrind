//
// Espresso mill controller
// (c) 2025 karl@pitrich.com
//
#include <stdio.h>

#include "esp_log.h"
#include "bsp/esp-box.h"
#include "lvgl.h"
#include "ui/ui.h"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "settings.h"

#define TAG "ESPGrind"
#define APP_DISP_DEFAULT_BRIGHTNESS 50

void app_lvgl_display(void) {
    bsp_display_lock(0);
    ui_init();
    bsp_display_unlock();
}

void nvs_init() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

#if 0
#define LV_TICK_PERIOD_MS 1
SemaphoreHandle_t xGuiSemaphore;

static void lv_tick_task(void *arg) {
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void guiTask(void *pvParameter) {

    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    bsp_i2c_init();
    bsp_display_start();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000);

    app_lvgl_display();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }

    vTaskDelete(NULL);
}
#endif 

void app_main(void)
{
    nvs_init();
    settingsLoad();
    
    bsp_i2c_init();
    bsp_display_start();

    app_lvgl_display();

    // xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);

    ESP_LOGI(TAG, "initialization done.");
}

