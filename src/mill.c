#include "mill.h"

static const mill_config_t MillConfig = {
    .relais_pin = GPIO_NUM_14,
    .on_level = 1
};

static mill_state_t MillState = { 0 };

void millInit() {
    gpio_set_direction(MillConfig.relais_pin, GPIO_MODE_OUTPUT);
    millOff();
}

void millOn() {
    MillState.is_milling = true;
    gpio_set_level(MillConfig.relais_pin, MillConfig.on_level);
}

void millOff() {
    MillState.is_milling = false;
    gpio_set_level(MillConfig.relais_pin, !MillConfig.on_level);
}

bool millIsMilling() {
    return MillState.is_milling;
}
