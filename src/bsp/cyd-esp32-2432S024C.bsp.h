
#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

/**************************************************************************************************
 *  BSP Capabilities
 */

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
// #define BSP_CAPS_SDCARD         1
// #define BSP_CAPS_IMU            1
// #define BSP_CAPS_AUDIO          1
// #define BSP_CAPS_AUDIO_SPEAKER  1
// #define BSP_CAPS_AUDIO_MIC      1
// #define BSP_CAPS_BUTTONS        1

/**************************************************************************************************/

// pinout 

#define BSP_I2C_SCL           (GPIO_NUM_18)
#define BSP_I2C_SDA           (GPIO_NUM_8)
#define BSP_I2C_NUM           (CONFIG_BSP_I2C_NUM)
#define I2C_SCL_IO            (10)        /*!< GPIO number used for I2C master clock */
#define I2C_SDA_IO            (11)        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BSP_LCD_SPI_NUM        (SPI2_HOST)
#define BSP_LCD_MISO           (GPIO_NUM_12)
#define BSP_LCD_MOSI           (GPIO_NUM_13)
#define BSP_LCD_SCLK           (GPIO_NUM_14)
#define BSP_LCD_CS             (GPIO_NUM_15)
#define BSP_LCD_DC             (GPIO_NUM_2)
#define BSP_LCD_RST            (GPIO_NUM_NC)

#define BSP_LCD_SPI_DMA_CHANNEL SPI_DMA_CH_AUTO
#define BSP_LCD_SPI_BUS_QUADWP_IO_NUM GPIO_NUM_NC
#define BSP_LCD_SPI_BUS_QUADHD_IO_NUM GPIO_NUM_NC


#define BSP_LCD_BACKLIGHT      (GPIO_NUM_27)
#define BSP_LCD_BACKLIGHT_ON    1
#define BSP_LCD_BK_LIGHT_OFF    !BSP_LCD_BK_LIGHT_ON_LEVEL

#define BSP_LCD_CMD_BITS           8
#define BSP_LCD_PARAM_BITS         8

#define ESP_LCD_COLOR_FORMAT_RGB565    (1)
#define ESP_LCD_COLOR_FORMAT_RGB888    (2)
#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB565)
#define BSP_LCD_COLOR_SPACE         (ESP_LCD_COLOR_SPACE_BGR)

#define BSP_LCD_DRAW_BUF_HEIGHT 100

#define BSP_LCD_PIXEL_CLOCK_HZ      (40 * 1000 * 1000)
#define BSP_LCD_BIGENDIAN           (0)
#define BSP_LCD_BITS_PER_PIXEL      (16)

#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (240)

// #define LVGL_BUFFER_PIXELS (BSP_LCD_H_RES*BSP_LCD_V_RES/4)
// #define BSP_LCD_SPI_BUS_MAX_TRANSFER_SZ (LVGL_BUFFER_PIXELS*sizeof(lv_color16_t))
#define BSP_LCD_SPI_BUS_MAX_TRANSFER_SZ ((BSP_LCD_H_RES * BSP_LCD_DRAW_BUF_HEIGHT) * sizeof(uint16_t))

#define LV_COLOR_16_SWAP 1

#define BSP_LCD_SWAP_XY false
#define BSP_LCD_MIRROR_X true
#define BSP_LCD_MIRROR_Y false


#define TOUCH_I2C_HOST I2C_NUM_0
#define TOUCH_I2C_CONFIG_SDA_IO_NUM (GPIO_NUM_33)
#define TOUCH_I2C_CONFIG_SCL_IO_NUM (GPIO_NUM_32)
#define TOUCH_CONFIG_RST_GPIO_NUM (GPIO_NUM_25)
#define TOUCH_CONFIG_INT_GPIO_NUM (GPIO_NUM_21)
#define TOUCH_I2C_CONFIG_SDA_PULLUP_EN GPIO_PULLUP_ENABLE
#define TOUCH_I2C_CONFIG_SCL_PULLUP_EN GPIO_PULLUP_ENABLE
#define TOUCH_I2C_CONFIG_MASTER_CLK_SPEED 400000
#define TOUCH_I2C_CONFIG_CLK_FLAGS 0
#define TOUCH_I2C_CONFIG_CONTROL_PHASE_BYTES 1
#define TOUCH_I2C_CONFIG_DC_BIT_OFFSET 0
#define TOUCH_I2C_CONFIG_LCD_CMD_BITS 8
#define TOUCH_I2C_CONFIG_LCD_PARAM_BITS 0
#define TOUCH_I2C_CONFIG_FLAGS_DC_LOW_ON_DATA false
#define TOUCH_I2C_CONFIG_FLAGS_DISABLE_CONTROL_PHASE true
#define TOUCH_CONFIG_X_MAX BSP_LCD_H_RES
#define TOUCH_CONFIG_Y_MAX BSP_LCD_V_RES

#define TOUCH_CONFIG_LEVELS_RESET 0
#define TOUCH_CONFIG_LEVELS_INTERRUPT 0
#define TOUCH_SWAP_XY true
#define TOUCH_MIRROR_X true
#define TOUCH_MIRROR_Y false

#define BSP_TF_CS 5
#define BSP_TF_SPI_MOSI 23
#define BSP_TF_SPI_SCLK 18
#define BSP_TF_SPI_MISO 19

#define BSP_RGB_LED_R 4
#define BSP_RGB_LED_G 16
#define BSP_RGB_LED_B 17

#define BSP_LDR_GPIO 34
// #define BSP_LDR_ADC_CHANNEL    ADC1_GPIO34_CHANNEL
#define BSP_LDR_ADC_ATTEN      ADC_ATTEN_DB_0


#define BSP_SPEAKER 26

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
    } flags;
} bsp_display_cfg_t;

/**
 * @brief Init I2C driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief Deinit I2C driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_deinit(void);

#define I2C_TRANS_BUF_MINIMUM_SIZE     (sizeof(i2c_cmd_desc_t) + \
                                        sizeof(i2c_cmd_link_t) * 8) /* It is required to have allocate one i2c_cmd_desc_t per command:
                                                                     * start + write (device address) + write buffer +
                                                                     * start + write (device address) + read buffer + read buffer for NACK +
                                                                     * stop */

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Get pointer to input device (touch, buttons, ...)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);


esp_err_t bsp_display_brightness_set(int brightness_percent);

void bsp_init();

float bsp_battery_get_voltage();
