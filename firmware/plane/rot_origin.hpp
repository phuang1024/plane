// Get rotational origin.

#pragma once

#include "sensors.hpp"
#include "utils.hpp"


/**
 * Returns imu's position.
 * Waits at least 8 seconds so ESC is initialized.
 * TODO make it smart and wait for the plane to stop moving for a while before measuring.
 * @param rot_origin  3d vector, xyz.
 */
void get_rot_origin(IMU& imu_sensor, float* rot_origin) {
    // Blink rapidly to tell user.
    LEDBlinker ledblink(13, 50, 150);
    int time_start = millis();
    while (millis() - time_start < 8000) {
        ledblink.update();
    }

    IMURead imu = imu_sensor.read();
    rot_origin[0] = imu.ax;
    rot_origin[1] = imu.ay;
    rot_origin[2] = imu.az;

    digitalWrite(13, LOW);
}
