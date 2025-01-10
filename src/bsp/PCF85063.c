/*
    PCF85063 driver
    Author: Waveshare Team
*/

#include <string.h>
#include "PCF85063.h"
#include "freertos/FreeRTOS.h"
#include "esp_check.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM              0         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS       1000

static esp_err_t bsp_rtc_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length);
static esp_err_t bsp_rtc_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length);
static uint8_t decToBcd(int val);
static int bcdToDec(uint8_t val);

const unsigned char monthStr[12][4] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov","Dec" };
const unsigned char dayOfWeekStr[7][4] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

void bsp_rtc_init(void)
{
	uint8_t data = RTC_CTRL_1_DEFAULT | RTC_CTRL_1_CAP_SEL;
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_CTRL_1_ADDR, &data, 1));
}

void bsp_rtc_reset(void)
{
	uint8_t value = RTC_CTRL_1_DEFAULT|RTC_CTRL_1_CAP_SEL|RTC_CTRL_1_SR;
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_CTRL_1_ADDR, &value, 1));
}

void bsp_rtc_set_time(datetime_t time)
{
	uint8_t buf[3] = { decToBcd(time.second),
					   decToBcd(time.minute),
					   decToBcd(time.hour) };
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_SECOND_ADDR, buf, 3));
}

void bsp_rtc_set_date(datetime_t date)
{
	uint8_t buf[4] = { decToBcd(date.day),
					   decToBcd(date.dotw),
					   decToBcd(date.month),
					   decToBcd(date.year - YEAR_OFFSET) };
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_DAY_ADDR, buf, 4));
}

void bsp_rtc_set_all(datetime_t time)
{
	uint8_t buf[7] = { decToBcd(time.second),
					   decToBcd(time.minute),
					   decToBcd(time.hour),
					   decToBcd(time.day),
					   decToBcd(time.dotw),
					   decToBcd(time.month),
					   decToBcd(time.year - YEAR_OFFSET) };
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_SECOND_ADDR, buf, 7));
}

void bsp_rtc_read_time(datetime_t *time)
{
	uint8_t buf[7] = { 0 };
	ESP_ERROR_CHECK(bsp_rtc_read(RTC_SECOND_ADDR, buf, 7));
	time->second = bcdToDec(buf[0] & 0x7F);
	time->minute = bcdToDec(buf[1] & 0x7F);
	time->hour   = bcdToDec(buf[2] & 0x3F);
	time->day    = bcdToDec(buf[3] & 0x3F);
	time->dotw   = bcdToDec(buf[4] & 0x07);
	time->month  = bcdToDec(buf[5] & 0x1F);
	time->year   = bcdToDec(buf[6]) + YEAR_OFFSET;
}

void bsp_rtc_alarm_enable(void)
{
	uint8_t Value = RTC_CTRL_2_DEFAULT | RTC_CTRL_2_AIE;
	Value &= ~RTC_CTRL_2_AF;
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_CTRL_2_ADDR, &Value, 1));
}

uint8_t bsp_rtc_alarm_get_flag(void)
{
	uint8_t value = 0;
	ESP_ERROR_CHECK(bsp_rtc_read(RTC_CTRL_2_ADDR, &value, 1));
	return value & (RTC_CTRL_2_AF | RTC_CTRL_2_AIE);
}

void bsp_rtc_alarm_set(datetime_t time)
{
	uint8_t buf[5] = {
		decToBcd(time.second) & (~RTC_ALARM),
		decToBcd(time.minute) & (~RTC_ALARM),
		decToBcd(time.hour)   & (~RTC_ALARM),
		RTC_ALARM, // disable day
		RTC_ALARM  // disable weekday
	};
	ESP_ERROR_CHECK(bsp_rtc_write(RTC_SECOND_ALARM, buf, 6));
}

void bsp_rtc_alarm_read(datetime_t *time)
{
	uint8_t bufss[6] = { 0 };
	ESP_ERROR_CHECK(bsp_rtc_read(RTC_SECOND_ALARM, bufss, 6));
	time->second = bcdToDec(bufss[0] & 0x7F);
	time->minute = bcdToDec(bufss[1] & 0x7F);
	time->hour   = bcdToDec(bufss[2] & 0x3F);
	time->day    = bcdToDec(bufss[3] & 0x3F);
	time->dotw   = bcdToDec(bufss[4] & 0x07);
}


static const uint8_t device_addr = PCF85063_ADDRESS;

static esp_err_t bsp_rtc_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length)
{
    uint8_t buf[length + 1];    
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, length);
    return i2c_master_write_to_device(I2C_MASTER_NUM, device_addr, buf, length+1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bsp_rtc_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, device_addr, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static uint8_t decToBcd(int val)
{
	return (uint8_t)((val / 10 * 16) + (val % 10));
}

static int bcdToDec(uint8_t val)
{
	return (int)((val / 16 * 10) + (val % 16));
}

void datetime_to_str(char *datetime_str, datetime_t time)
{
	sprintf(datetime_str, "%s %04d.%02d.%02d %02d:%02d:%02d",
                        dayOfWeekStr[time.dotw], 
                        time.year, time.month, time.day,
                        time.hour, time.minute, time.second);
}
