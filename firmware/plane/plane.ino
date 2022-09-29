#include <Servo.h>
#include <Wire.h>

#include "pid.hpp"
#include "sensors.hpp"
#include "utils.hpp"


/**
 * Waits until IMU is not fluctuating (plane is set on floor) and gets rot position.
 */
void get_rot_origin(IMU& imu_sensor, int* rot_origin) {
}


void setup() {
    Serial.begin(9600);
    Wire.begin();

    IMU imu_sensor;
    Baro baro_sensor;
    imu_sensor.init();
    baro_sensor.init();

    // Find (0, 0, 0) of rotational position.
    int rot_origin[3];
    get_rot_origin(imu_sensor, rot_origin);

    Servo servo;
    servo.attach(2);
    PIDControl pidctrl(3, 0.1, 0.004, 0);

    LEDBlinker ledblink(13, 50, 950);

    while (true) {
        IMURead imu = imu_sensor.read();
        //BaroRead baro = baro_sensor.read();

        float value = (float)imu.ax / 65536 * 2*PI;
        float ctrl = pidctrl.control(value);
        int angle = constrain(mapf(ctrl, -1, 1, 0, 180), 0, 180);
        Serial.println(angle);
        servo.write(angle);

        ledblink.update();

        delay(20);
    }
}


void loop() {
}
