#include "display.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "display";

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_band[2];
static int s_next;
static SemaphoreHandle_t s_free;

static bool on_color_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)io;
    (void)edata;
    (void)user_ctx;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_free, &hp);
    return hp == pdTRUE;
}

esp_err_t display_init(void)
{
    s_free = xSemaphoreCreateCounting(2, 2);
    if (!s_free) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < 2; i++) {
        s_band[i] = heap_caps_calloc((size_t)LCD_H_RES * BAND_ROWS, sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_band[i]) {
            return ESP_ERR_NO_MEM;
        }
    }

    const spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PCLK,
        .data0_io_num = LCD_D0,
        .data1_io_num = LCD_D1,
        .data2_io_num = LCD_D2,
        .data3_io_num = LCD_D3,
        .max_transfer_sz = LCD_H_RES * BAND_ROWS * sizeof(uint16_t) + 16,
        .flags = SPICOMMON_BUSFLAG_QUAD,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus");

    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = -1,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_color_trans_done,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .flags = {
            .quad_mode = 1,
        },
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io), TAG, "io");

    co5300_vendor_config_t vendor = {
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(io, &panel_config, &s_panel), TAG, "panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, LCD_COL_OFFSET, 0), TAG, "gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "on");
    ESP_LOGI(TAG, "466x466 CO5300 ready");
    return ESP_OK;
}

uint16_t *display_acquire_band(void)
{
    xSemaphoreTake(s_free, portMAX_DELAY);
    uint16_t *buf = s_band[s_next];
    s_next ^= 1;
    return buf;
}

esp_err_t display_flush_band(int y0, int rows, const uint16_t *buffer)
{
    return esp_lcd_panel_draw_bitmap(s_panel, 0, y0, LCD_H_RES, y0 + rows, buffer);
}

void display_frame(void (*draw)(uint16_t *band, int y0, int rows))
{
    for (int y = 0; y < LCD_V_RES; y += BAND_ROWS) {
        int rows = BAND_ROWS;
        if (y + rows > LCD_V_RES) {
            rows = LCD_V_RES - y;
        }
        uint16_t *band = display_acquire_band();
        draw(band, y, rows);
        ESP_ERROR_CHECK(display_flush_band(y, rows, band));
    }
}
