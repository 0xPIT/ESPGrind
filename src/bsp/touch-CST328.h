#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_LCD_TOUCH_IO_I2C_CST328_ADDRESS (0x1A)

#define CST328_REG_DEBUG_INFO_MODE              (0xD101)
#define CST328_REG_RESET_MODE            	    (0xD102)
#define CST328_REG_REDO_RECALIBRATION           (0xD104)
#define CST328_REG_DEEP_SLEEP_MODE    		    (0xD105)
#define CST328_REG_DEBUG_POINT_MODE	    	    (0xD108)
#define CST328_REG_NORMAL_MODE                  (0xD109)

#define CST328_REG_DEBUG_RAWDATA_MODE           (0xD10A)
#define CST328_REG_DEBUG_DIFF_MODE              (0xD10D)
#define CST328_REG_DEBUG_FACTORY_MODE           (0xD119)
#define CST328_REG_DEBUG_FACTORY_MODE_2         (0xD120)

#define CST328_REG_DEBUG_INFO_BOOT_TIME         (0xD1FC)
#define CST328_REG_DEBUG_INFO_RES_Y             (0xD1FA)
#define CST328_REG_DEBUG_INFO_RES_X             (0xD1F8)
#define CST328_REG_DEBUG_INFO_KEY_NUM           (0xD1F7)
#define CST328_REG_DEBUG_INFO_TP_NRX            (0xD1F6)
#define CST328_REG_DEBUG_INFO_TP_NTX            (0xD1F4)

#define ESP_LCD_TOUCH_CST328_TOUCHREPORT        (0xD000)
#define ESP_LCD_TOUCH_CST328_READ_FINGER_REG    (0xD005)
#define ESP_LCD_TOUCH_CST328_READ_CHECK_REG     (0xD006) // fixed value 0xAB

#define ESP_LCD_TOUCH_CST328_CHECK_BYTE           (0xAB)

typedef struct {
    uint8_t status : 4;   // Low 4 bits -> Status
    uint8_t id : 4;       // High 4 bits -> ID
    uint8_t x_hi;               // High 8 bits of X resolution
    uint8_t y_hi;               // High 8 bits of Y resolution
    uint8_t y_lo : 4;    // Low 4 bits of Y resolution
    uint8_t x_lo : 4;    // Low 4 bits of X resolution
    uint8_t pressure;            // Pressure
} __attribute__((packed)) touch_finger_t;

typedef struct {
    touch_finger_t finger1;   // Data for the 1st finger
    uint8_t key_report_flag;  // 0xD005: Key report flag -> & with 0x7F to get the number of fingers currently touched
    uint8_t check_value;      // 0xD006: Fixed value (0xAB)
#if CONFIG_ESP_LCD_TOUCH_MAX_POINTS > 1
    touch_finger_t fingers[CONFIG_ESP_LCD_TOUCH_MAX_POINTS - 1];
#endif
} __attribute__((packed)) touch_report_t;


// I2C settings
#define I2C_Touch_SCL_IO            3      /*!< GPIO number used for I2C master clock */
#define I2C_Touch_SDA_IO            1      /*!< GPIO number used for I2C master data  */
#define I2C_Touch_INT_IO            4      /*!< GPIO number used for I2C master data  */
#define I2C_Touch_RST_IO            2      /*!< GPIO number used for I2C master clock */
#define I2C_Touch_MASTER_NUM        1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

/**
 * @brief Touch IO configuration structure
 *
 */
#define ESP_LCD_TOUCH_IO_I2C_CST328_CONFIG()           \
    {                                       \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST328_ADDRESS, \
        .control_phase_bytes = 1,           \
        .dc_bit_offset = 0,                 \
        .lcd_cmd_bits = 16,                 \
        .flags =                            \
        {                                   \
            .disable_control_phase = 1,     \
        }                                   \
    }

esp_err_t esp_lcd_touch_new_i2c_cst328(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch);

#ifdef __cplusplus
}
#endif