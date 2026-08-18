#include "pattern.h"
#include "config.h"

#include <math.h>
#include <string.h>

/* Pixel-centers of 0..465 sit on 232.5. Radius 232.5 is the inscribed circle. */
#define CX 232.5f
#define CY 232.5f
#define R_MAX 232.5f
#define RING_MID (R_MAX - 2.0f)
#define RING_HALF 1.6f
#define CROSS_LIMIT (RING_MID + RING_HALF)

static uint16_t rgb565(int r, int g, int b)
{
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c << 8) | (c >> 8));
}

void pattern_draw_band(uint16_t *band, int y0, int rows)
{
    const uint16_t white = rgb565(255, 255, 255);

    memset(band, 0, (size_t)LCD_H_RES * (size_t)rows * sizeof(uint16_t));

    for (int row = 0; row < rows; row++) {
        const int y = y0 + row;
        uint16_t *dst = band + row * LCD_H_RES;
        const float fy = (float)y + 0.5f;

        for (int x = 0; x < LCD_H_RES; x++) {
            const float fx = (float)x + 0.5f;
            const float r = hypotf(fx - CX, fy - CY);
            if (r > R_MAX) {
                continue;
            }

            const int on_cross = (fabsf(fx - CX) < 1.5f || fabsf(fy - CY) < 1.5f) &&
                                 r <= CROSS_LIMIT;
            const int on_ring = fabsf(r - RING_MID) <= RING_HALF;
            if (on_cross || on_ring) {
                dst[x] = white;
            }
        }
    }
}
