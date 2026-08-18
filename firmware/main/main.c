#include "button.h"
#include "config.h"
#include "display.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "render.h"
#include "sim.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_14
#define BUTTON_PERIOD_MS 25

static const char *TAG = "fluidbox";

static i2c_master_bus_handle_t s_i2c_bus;
static volatile uint32_t s_frames;
static volatile uint32_t s_steps;

static esp_err_t i2c_init(void)
{
    const i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &s_i2c_bus);
}

static void render_task(void *arg)
{
    (void)arg;
    for (;;) {
        render_frame();
        s_frames++;
        vTaskDelay(1);
    }
}

static void sim_task(void *arg)
{
    (void)arg;
    sim_forces_t forces = {
        .gravity = {0.0f, GRAVITY_GAIN * GRAVITY_MPS2 * PX_PER_METER, 0.0f},
        .omega = {0.0f, 0.0f, 0.0f},
        .alpha = {0.0f, 0.0f, 0.0f},
    };
    int64_t last_us = esp_timer_get_time();
    int64_t last_button_us = last_us;

    for (;;) {
        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (dt > 0.05f) {
            dt = 0.05f;
        } else if (dt < 1e-4f) {
            dt = 1e-4f;
        }

        imu_read(dt, &forces);
        sim_step(dt, &forces);
        s_steps++;

        if (now - last_button_us >= BUTTON_PERIOD_MS * 1000) {
            last_button_us = now;
            if (button_take_short_press()) {
                ESP_LOGI(TAG, "PWR pressed, resetting fluid");
                sim_reset();
            }
        }

        vTaskDelay(1);
    }
}

static void stats_loop(void)
{
    uint32_t last_frames = 0, last_steps = 0;
    int64_t last_us = esp_timer_get_time();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        const int64_t now = esp_timer_get_time();
        const float elapsed = (float)(now - last_us) * 1e-6f;
        last_us = now;

        const uint32_t frames = s_frames;
        const uint32_t steps = s_steps;
        sim_stats_t st;
        sim_stats(&st);
        float accel[3];
        imu_raw_accel(accel);

        ESP_LOGI(TAG,
                 "%.1f fps | %.1f steps/s | rho %.2f/%.2f | speed avg %.0f max %.0f | "
                 "accel %.2f %.2f %.2f | sram %u",
                 (double)((float)(frames - last_frames) / elapsed),
                 (double)((float)(steps - last_steps) / elapsed),
                 (double)st.mean_density, (double)st.rest_density,
                 (double)st.mean_speed, (double)st.max_speed,
                 (double)accel[0], (double)accel[1], (double)accel[2],
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

        last_frames = frames;
        last_steps = steps;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "FluidBox layer 4");
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(display_init());

    if (imu_init(s_i2c_bus) != ESP_OK) {
        ESP_LOGW(TAG, "continuing without motion input");
    }
    if (button_init(s_i2c_bus) != ESP_OK) {
        ESP_LOGW(TAG, "continuing without the reset button");
    }

    sim_init();
    render_init();
    xTaskCreatePinnedToCore(sim_task, "sim", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 0);
    stats_loop();
}
