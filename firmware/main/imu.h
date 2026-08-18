#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "sim.h"

// Turns QMI8658 samples into the forces acting on the fluid.
//
// An accelerometer at rest reads the reaction to gravity, so the low-pass of
// its output points "up" and the remainder is whatever the hand is doing.
// Splitting the signal that way gives tilt and shake independently.

esp_err_t imu_init(i2c_master_bus_handle_t bus);

// Samples the IMU and fills in gravity, angular velocity and angular
// acceleration for the next simulation step. Returns false if the sensor did
// not have fresh data, in which case the previous forces are reused.
bool imu_read(float dt, sim_forces_t *out);

// Most recent raw accelerometer reading in m/s^2, for the startup log that
// confirms the axis mapping.
void imu_raw_accel(float out[3]);

/* Unit vector of gravity in sim space (where the puddle should sit). */
void imu_down(float out[3]);
