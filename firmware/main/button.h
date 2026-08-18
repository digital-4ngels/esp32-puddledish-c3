#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t button_init(i2c_master_bus_handle_t bus);
bool button_take_short_press(void);
