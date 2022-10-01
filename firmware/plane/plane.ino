#include <RH_ASK.h>
#include <ServoTimer2.h>
#include <Wire.h>

#include "pid.hpp"
#include "rot_origin.hpp"
#include "sensors.hpp"
#include "utils.hpp"

const bool CALIBRATE_MODE = false;


void setup() {
    Serial.begin(9600);
    Wire.begin();

    IMU imu_sensor;
    Baro baro_sensor;
    imu_sensor.init();
    baro_sensor.init();

    ServoTimer2 motor;
    ServoTimer2 servo;
    motor.attach(2);
    servo.attach(3);
    motor.write(700);

    // Calibrate if calibrate mode.
    if (CALIBRATE_MODE) {
        Serial.println("Calibrate mode.");

        // Calibrate motor (high then low).
        motor.write(2300);
        delay(7000);
        motor.write(700);

        while (true);  // Don't do anything else.
    }

    // Find (0, 0, 0) of rotational position.
    // The delay here also makes sure ESC is initialized.
    float rot_origin[3];
    get_rot_origin(imu_sensor, rot_origin);

    PIDControl pidctrl(3, 0.2, 0.004, rot_origin[0]);
    LEDBlinker ledblink(13, 50, 1950);

    // TODO temporary radio test
    /*
    Serial.println("start");
    servo.write(700);
    delay(300);
    servo.write(2300);
    delay(500);
    RH_ASK radio;
    if (!radio.init()) {
        Serial.println("Radio init failed.");
    }
    while (true) {
        uint8_t msg[12];
        uint8_t msglen = sizeof(msg);
        if (radio.recv(msg, &msglen)) {
            int i;
            Serial.println("recv");
            servo.write(700);
            delay(100);
            servo.write(2300);
            delay(100);
        }
    }
    */

    motor.write(1000);

    while (true) {
        IMURead imu = imu_sensor.read();
        //BaroRead baro = baro_sensor.read();

        float value = (float)imu.ax;
        float ctrl = pidctrl.control(value);
        int angle = constrain(mapf(ctrl, -1, 1, 700, 2300), 700, 2300);
        Serial.println(angle);
        servo.write(angle);

        ledblink.update();

        delay(20);
    }
}


void loop() {
}
