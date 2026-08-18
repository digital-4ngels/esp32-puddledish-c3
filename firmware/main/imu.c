#include "imu.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// qmi8658.h defines its own single-precision M_PI, which collides with the one
// math.h already provided. Drop ours first; this file uses TWO_PI below.
#undef M_PI
#include "qmi8658.h"

#define IMU_PROBE_TIMEOUT_MS 100
#define QMI8658_RESET_REGISTER 0x60
#define QMI8658_RESET_COMMAND 0xB0
#define QMI8658_CTRL1_VALUE 0x60
#define QMI8658_RESET_DELAY_MS 20

#define DEG_TO_RAD 0.01745329252f
#define TWO_PI 6.28318530718f

static const char *TAG = "imu";

static qmi8658_dev_t s_dev;
static bool s_ready;

static float s_lp[3];  // low-passed accelerometer, i.e. the gravity part
static bool s_lp_primed;
static float s_prev_omega[3];
static float s_raw_accel[3];
static float s_down[3] = {0.0f, 1.0f, 0.0f};

static esp_err_t detect_address(i2c_master_bus_handle_t bus, uint8_t *address)
{
    const uint8_t candidates[] = {QMI8658_ADDRESS_HIGH, QMI8658_ADDRESS_LOW};

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        if (i2c_master_probe(bus, candidates[i], IMU_PROBE_TIMEOUT_MS) == ESP_OK) {
            *address = candidates[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t configure(void)
{
    esp_err_t ret = qmi8658_write_register(&s_dev, QMI8658_RESET_REGISTER, QMI8658_RESET_COMMAND);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(QMI8658_RESET_DELAY_MS));

    ret = qmi8658_write_register(&s_dev, QMI8658_CTRL1, QMI8658_CTRL1_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }

    // 8g covers a hard shake without clipping; 250 Hz is comfortably above the
    // simulation rate so there is always a fresh sample.
    if ((ret = qmi8658_set_accel_range(&s_dev, QMI8658_ACCEL_RANGE_8G)) != ESP_OK) return ret;
    if ((ret = qmi8658_set_accel_odr(&s_dev, QMI8658_ACCEL_ODR_250HZ)) != ESP_OK) return ret;
    if ((ret = qmi8658_set_gyro_range(&s_dev, QMI8658_GYRO_RANGE_512DPS)) != ESP_OK) return ret;
    if ((ret = qmi8658_set_gyro_odr(&s_dev, QMI8658_GYRO_ODR_250HZ)) != ESP_OK) return ret;

    qmi8658_set_accel_unit_mps2(&s_dev, true);
    qmi8658_set_gyro_unit_dps(&s_dev, true);

    return qmi8658_enable_sensors(&s_dev, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
}

esp_err_t imu_init(i2c_master_bus_handle_t bus)
{
    uint8_t address = 0;
    esp_err_t ret = detect_address(bus, &address);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QMI8658 not found on I2C");
        return ret;
    }

    ret = qmi8658_init(&s_dev, bus, address);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "configure failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "QMI8658 ready at 0x%02x", address);
    s_ready = true;
    return ESP_OK;
}

void imu_raw_accel(float out[3])
{
    memcpy(out, s_raw_accel, sizeof(s_raw_accel));
}

bool imu_read(float dt, sim_forces_t *out)
{
    if (!s_ready) {
        return false;
    }

    bool data_ready = false;
    if (qmi8658_is_data_ready(&s_dev, &data_ready) != ESP_OK || !data_ready) {
        return false;
    }

    qmi8658_data_t data;
    if (qmi8658_read_sensor_data(&s_dev, &data) != ESP_OK) {
        return false;
    }

    s_raw_accel[0] = data.accelX;
    s_raw_accel[1] = data.accelY;
    s_raw_accel[2] = data.accelZ;

    // Move into simulation axes: x right, y down the screen, z into the case.
    const float ax = IMU_MAP_X(data.accelX, data.accelY, data.accelZ);
    const float ay = IMU_MAP_Y(data.accelX, data.accelY, data.accelZ);
    const float az = IMU_MAP_Z(data.accelX, data.accelY, data.accelZ);

    const float gx = IMU_MAP_X(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
    const float gy = IMU_MAP_Y(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
    const float gz = IMU_MAP_Z(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;

    if (!s_lp_primed) {
        s_lp[0] = ax;
        s_lp[1] = ay;
        s_lp[2] = az;
        s_lp_primed = true;
    } else {
        // One-pole low pass. The coefficient is derived from the cutoff so the
        // feel does not change if the loop rate does.
        const float k = 1.0f - expf(-TWO_PI * GRAVITY_LP_HZ * dt);
        s_lp[0] += k * (ax - s_lp[0]);
        s_lp[1] += k * (ay - s_lp[1]);
        s_lp[2] += k * (az - s_lp[2]);
    }

    // The steady part of the signal is the reaction to gravity, so gravity
    // itself points the opposite way.
    float dx = -s_lp[0], dy = -s_lp[1], dz = -s_lp[2];
    const float mag = sqrtf(dx * dx + dy * dy + dz * dz);
    if (mag > 0.5f) {
        dx /= mag;
        dy /= mag;
        dz /= mag;
    } else {
        dx = 0.0f;
        dy = 0.0f;
        dz = 1.0f;
    }
    s_down[0] = dx;
    s_down[1] = dy;
    s_down[2] = dz;

    const float g_px = GRAVITY_GAIN * GRAVITY_MPS2 * PX_PER_METER;

    // What is left after removing gravity is the box's own acceleration. In
    // the box's frame the contents feel that as a push the other way.
    const float shake_x = ax - s_lp[0];
    const float shake_y = ay - s_lp[1];
    const float shake_z = az - s_lp[2];
    const float shake_px = SHAKE_GAIN * PX_PER_METER;

    out->gravity[0] = dx * g_px - shake_x * shake_px;
    out->gravity[1] = dy * g_px - shake_y * shake_px;
    out->gravity[2] = dz * g_px - shake_z * shake_px;

    out->omega[0] = gx;
    out->omega[1] = gy;
    out->omega[2] = gz;

    const float inv_dt = (dt > 1e-6f) ? (1.0f / dt) : 0.0f;
    out->alpha[0] = (gx - s_prev_omega[0]) * inv_dt;
    out->alpha[1] = (gy - s_prev_omega[1]) * inv_dt;
    out->alpha[2] = (gz - s_prev_omega[2]) * inv_dt;

    s_prev_omega[0] = gx;
    s_prev_omega[1] = gy;
    s_prev_omega[2] = gz;

    return true;
}

void imu_down(float out[3])
{
    out[0] = s_down[0];
    out[1] = s_down[1];
    out[2] = s_down[2];
}
