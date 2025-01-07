#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

typedef struct {
    gpio_num_t relais_pin;
    uint32_t on_level;
} mill_config_t;

typedef struct {
    bool is_milling;
} mill_state_t;

void millInit();
void millOn();
void millOff();
bool millIsMilling();
