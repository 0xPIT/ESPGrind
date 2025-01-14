#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"

#include "touch-CST328.h"

static const char *TAG = "CST328";

static const uint8_t clear = 0x00;

const size_t report_size = sizeof(touch_report_t);
touch_report_t *report;

static esp_err_t esp_lcd_touch_cst328_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_cst328_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_cst328_del(esp_lcd_touch_handle_t tp);

static esp_err_t touch_cst328_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_cst328_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, const uint8_t *data, uint8_t len);
static esp_err_t touch_cst328_reset(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_cst328(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(io != NULL);
    assert(config != NULL);
    assert(out_touch != NULL);

    esp_lcd_touch_handle_t esp_lcd_touch_cst328 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_cst328, ESP_ERR_NO_MEM, err, TAG, "Failed to allocate memory for CST328 controller");

    esp_lcd_touch_cst328->io = io;

    esp_lcd_touch_cst328->read_data = esp_lcd_touch_cst328_read_data;
    esp_lcd_touch_cst328->get_xy = esp_lcd_touch_cst328_get_xy;
    esp_lcd_touch_cst328->del = esp_lcd_touch_cst328_del;
    esp_lcd_touch_cst328->data.lock.owner = portMUX_FREE_VAL;

    // save config
    memcpy(&esp_lcd_touch_cst328->config, config, sizeof(esp_lcd_touch_config_t));

    // prepare pin for touch interrupt
    if (esp_lcd_touch_cst328->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = BIT64(esp_lcd_touch_cst328->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
        
        // register interrupt callback
        if (esp_lcd_touch_cst328->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_cst328, esp_lcd_touch_cst328->config.interrupt_callback);
        }
    }

    // reset controller
    ret = touch_cst328_reset(esp_lcd_touch_cst328);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "CST328 reset failed");
    touch_cst328_i2c_write(esp_lcd_touch_cst328, CST328_REG_NORMAL_MODE, &clear, 0);

    // allocate memory for touch report
    report = (touch_report_t *)malloc(report_size);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_cst328, ESP_ERR_NO_MEM, err, TAG, "Failed to allocate memory for raw report");

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller CST328 initialization failed!", ret);
        if (esp_lcd_touch_cst328) {
            esp_lcd_touch_cst328_del(esp_lcd_touch_cst328);
        }
    }

    *out_touch = esp_lcd_touch_cst328;

    return ret;
}

static esp_err_t esp_lcd_touch_cst328_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t points = 0;

    assert(tp != NULL);

    err = touch_cst328_i2c_read(tp, ESP_LCD_TOUCH_CST328_TOUCHREPORT, (uint8_t *)report, sizeof(touch_finger_t) + 2);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    if (report->check_value != ESP_LCD_TOUCH_CST328_CHECK_BYTE) {
        ESP_LOGD(TAG, "touch report data not valid\n");
        touch_cst328_i2c_write(tp, ESP_LCD_TOUCH_CST328_READ_FINGER_REG, &clear, 0);
        return ESP_OK;
    }

    points = report->key_report_flag & 0x7F;
    if (points == 0) {
        ESP_LOGD(TAG, "no touch points found\n");
        touch_cst328_i2c_write(tp, ESP_LCD_TOUCH_CST328_READ_FINGER_REG, &clear, 0);
        return ESP_OK;
    }
    points = (points > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : points);

    taskENTER_CRITICAL(&tp->data.lock);
        tp->data.points = points;
        for (uint8_t i = 0; i < points; i++) {
            touch_finger_t *finger = &report->finger1;
            if (i == 0) {
                finger = &report->finger1;
            } 
            #if CONFIG_ESP_LCD_TOUCH_MAX_POINTS > 1
            else {
                finger = &report->fingers[i-1];
            }
            #endif

            tp->data.coords[i].x = (finger->x_hi << 4) | finger->x_lo;
            tp->data.coords[i].y = (finger->y_hi << 4) | finger->y_lo;
            tp->data.coords[i].strength = finger->pressure;
        }   
    taskEXIT_CRITICAL(&tp->data.lock);

    // ------------------------------------------------------------------------------------------------------------------------------
    // playground
    //
    #if 0
    bool pressed = report->finger1.status & 0x04;
    uint16_t xx = (report->finger1.x_hi << 4) | report->finger1.x_lo;
    uint16_t yy = (report->finger1.y_hi << 4) | report->finger1.y_lo;    
    //if (points > 0)
        printf("[cst328][info] fingers = %d, fingerId = 0x%02x, strength = %d, status = 0x%02x, pressed = %d, xx = %d, yy = %d\n", 
                points, report->finger1.id, report->finger1.pressure, report->finger1.status, pressed, xx, yy);
    #endif
    // ------------------------------------------------------------------------------------------------------------------------------
    
    touch_cst328_i2c_write(tp, ESP_LCD_TOUCH_CST328_READ_FINGER_REG, &clear, 0);

    return ESP_OK;
}

static bool esp_lcd_touch_cst328_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    taskENTER_CRITICAL(&tp->data.lock);
        if (tp->data.points > max_point_num) {
            tp->data.points = max_point_num;
        }

        for (uint8_t i = 0; i < tp->data.points; i++) {
            x[i] = tp->data.coords[i].x;
            y[i] = tp->data.coords[i].y;

            if (strength) {
                strength[i] = tp->data.coords[i].strength;
            }
        }
        *point_num = tp->data.points;

        tp->data.points = 0; // invalidate

    taskEXIT_CRITICAL(&tp->data.lock);

    return *point_num > 0;
}

static esp_err_t esp_lcd_touch_cst328_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);
    free(report);

    return ESP_OK;
}

static esp_err_t touch_cst328_reset(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset);
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

#if 0
static void touch_cst328_read_cfg(esp_lcd_touch_handle_t tp)
{
    uint8_t buf[24];
    assert(tp != NULL);
    touch_cst328_i2c_write(tp, CST328_REG_DEBUG_INFO_MODE, &clear, 1);
    touch_cst328_i2c_read(tp, CST328_REG_DEBUG_INFO_BOOT_TIME, (uint8_t *)&buf[0], 4);
    ESP_LOGI(TAG, "TouchPad_ID:0x%02x,0x%02x,0x%02x,0x%02x", buf[0], buf[1], buf[2], buf[3]);

    touch_cst328_i2c_read(tp, CST328_REG_DEBUG_INFO_RES_X, (uint8_t *)&buf[0], 1);
    touch_cst328_i2c_read(tp, CST328_REG_DEBUG_INFO_RES_X+1, (uint8_t *)&buf[1], 1);
    ESP_LOGI(TAG, "TouchPad_X_MAX:%d", buf[1]*256+buf[0]);
    touch_cst328_i2c_read(tp, CST328_REG_DEBUG_INFO_RES_Y, (uint8_t *)&buf[2], 1);
    touch_cst328_i2c_read(tp, CST328_REG_DEBUG_INFO_RES_Y+1, (uint8_t *)&buf[3], 1);
    ESP_LOGI(TAG, "TouchPad_Y_MAX:%d", buf[3]*256+buf[2]);
    
    touch_cst328_i2c_read(tp, CST328_REG_DEBUG_INFO_TP_NTX, buf, 24);
    ESP_LOGI(TAG, "D1F4:0x%02x,0x%02x,0x%02x,0x%02x", buf[0], buf[1], buf[2], buf[3]);
    ESP_LOGI(TAG, "D1F8:0x%02x,0x%02x,0x%02x,0x%02x", buf[4], buf[5], buf[6], buf[7]);
    ESP_LOGI(TAG, "D1FC:0x%02x,0x%02x,0x%02x,0x%02x", buf[8], buf[9], buf[10], buf[11]);
    ESP_LOGI(TAG, "D200:0x%02x,0x%02x,0x%02x,0x%02x", buf[12], buf[13], buf[14], buf[15]);
    ESP_LOGI(TAG, "D204:0x%02x,0x%02x,0x%02x,0x%02x", buf[16], buf[17], buf[18], buf[19]);
    ESP_LOGI(TAG, "D208:0x%02x,0x%02x,0x%02x,0x%02x", buf[20], buf[21], buf[22], buf[23]);
}
#endif

static esp_err_t touch_cst328_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    assert(tp != NULL);
    assert(data != NULL);
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_cst328_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, const uint8_t* data, uint8_t len)
{
    assert(tp != NULL);
    return esp_lcd_panel_io_tx_param(tp->io, reg, data, len);
}
