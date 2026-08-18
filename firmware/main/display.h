#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t display_init(void);
uint16_t *display_acquire_band(void);
esp_err_t display_flush_band(int y0, int rows, const uint16_t *buffer);
void display_frame(void (*draw)(uint16_t *band, int y0, int rows));
