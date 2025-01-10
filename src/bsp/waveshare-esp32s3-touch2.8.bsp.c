#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lvgl_port.h"

#include "bsp/waveshare-esp32s3-touch2.8.bsp.h"
#include "bsp/touch-CST328.h"
#include "bsp/ST7789T.h"

static const char *TAG = "WS-ESP32S3-TOUCH2.8";

typedef struct {
    void *dummy;    /*!< Prepared for future use. */
} bsp_touch_config_t;


typedef struct {
    int max_transfer_sz;    /*!< Maximum transfer size, in bytes. */
} bsp_display_config_t;

static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
static esp_lcd_touch_handle_t tp;
static bool i2c_initialized = false;

esp_err_t bsp_i2c_init(void)
{
    if (i2c_initialized) {
        return ESP_OK;
    }

    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    i2c_initialized = true;
    return ESP_OK;
}

esp_err_t touch_i2c_init(void)
{
    int i2c_master_port = I2C_Touch_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_Touch_SDA_IO,
        .scl_io_num = I2C_Touch_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t bsp_i2c_deinit(void)
{
    i2c_driver_delete(I2C_MASTER_NUM);
    i2c_initialized = false;
    return ESP_OK;
}

esp_err_t bsp_display_brightness_init(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_LCD_BACKLIGHT
    };
    gpio_config(&bk_gpio_config);

    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };

    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&LCD_backlight_timer);
    ledc_channel_config(&LCD_backlight_channel);

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) brightness_percent = 100;
    if (brightness_percent < 0)   brightness_percent = 0;

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = ( ((1<<LEDC_TIMER_13_BIT) - 1) * brightness_percent) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {                                                         
        .sclk_io_num = BSP_LCD_SCLK,                                            
        .mosi_io_num = BSP_LCD_MOSI,                                            
        .miso_io_num = BSP_LCD_MISO,                                            
        .quadwp_io_num = GPIO_NUM_NC,                                                            
        .quadhd_io_num = GPIO_NUM_NC,                                                            
        .max_transfer_sz = BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t),    
    };
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));   

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {                                             
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = BSP_LCD_CMD_BITS,
        .lcd_param_bits = BSP_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io));

    ESP_LOGD(TAG, "Install LCD panel");    
    const esp_lcd_panel_dev_st7789t_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST, // Shared with Touch reset
        .color_space = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789t(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_mirror(*ret_panel, true, false);

    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = (BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT) * sizeof(uint16_t),
    };
    bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle);

    esp_lcd_panel_disp_on_off(panel_handle, true);

    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy  = true,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    touch_i2c_init(); // uses different pins

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_V_RES,
        .y_max = BSP_LCD_H_RES,
        .rst_gpio_num = I2C_Touch_RST_IO,
        .int_gpio_num = GPIO_NUM_NC, // I2C_Touch_INT_IO, // too fast using interrupt
        .flags = {
            .swap_xy  = true,
            .mirror_x = true,
            .mirror_y = false,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST328_CONFIG();
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_Touch_MASTER_NUM, &tp_io_config, &tp_io_handle);

    return esp_lcd_touch_new_i2c_cst328(tp_io_handle, &tp_cfg, ret_touch);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    bsp_touch_new(NULL, &tp);
    assert(tp);

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    lvgl_port_init(&cfg->lvgl_port_cfg);
    bsp_display_brightness_init();
    disp = bsp_display_lcd_init(cfg);
    disp_indev = bsp_display_indev_init(disp);

    return disp;
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
#if CONFIG_BSP_LCD_DRAW_BUF_DOUBLE
        .double_buffer = 1,
#else
        .double_buffer = 0,
#endif
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

static bool battery_initialized = false;
static adc_oneshot_unit_handle_t bsp_battery_adc_handle;
static bool bsp_battery_calibrated = false;
static adc_cali_handle_t bsp_battery_adc_calibration = NULL;

static bool bsp_battery_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } 
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } 
    else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void bsp_battery_init() 
{
    adc_oneshot_unit_init_cfg_t init_config = {                          
        .unit_id = ADC_UNIT_1,                                               
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &bsp_battery_adc_handle));      

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = BSP_BATTERY_ADC_ATTEN,                                               
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(bsp_battery_adc_handle, BSP_BATTERY_ADC_CHANNEL, &chan_config)); 

    bsp_battery_calibrated = bsp_battery_adc_calibration_init(ADC_UNIT_1, BSP_BATTERY_ADC_CHANNEL, BSP_BATTERY_ADC_ATTEN, &bsp_battery_adc_calibration);

    battery_initialized = true;
}   

float bsp_battery_get_voltage() {
    static const float battery_measurement_offset = 0.994500; // #define Measurement_offset 0.994500  
    static float analogVolts = 0;

    static int adc_raw[2][10];                           
    static int voltage[2][10];                          

    adc_oneshot_read(bsp_battery_adc_handle, BSP_BATTERY_ADC_CHANNEL, &adc_raw[0][0]);                                                     
    if (bsp_battery_calibrated) {                                                                                           
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(bsp_battery_adc_calibration, adc_raw[0][0], &voltage[0][0]));                    
        analogVolts = (float)(voltage[0][0] * 3.0 / 1000.0) / battery_measurement_offset;
    }

    return analogVolts;
}