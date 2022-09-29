#pragma once

#include <Wire.h>

const int I2C_IMU = 0x68;
const int I2C_BARO = 0x77;


float mapf(float v, float old_min, float old_max, float new_min, float new_max) {
    return (v-old_min) / (old_max-old_min) * (new_max-new_min) + new_min;
}

float constrainf(float v, float min_v, float max_v) {
    return min(max(v, min_v), max_v);
}


uint16_t read_u16() {
    uint16_t x = 0;
    for (int i = 0; i < 2; i++) {
        x = (x << 8) | Wire.read();
    }
    return x;
}

uint32_t read_u24() {
    uint32_t x = 0;
    for (int i = 0; i < 3; i++) {
        x = (x << 8) | Wire.read();
    }
    return x;
}


/**
 * Manages all the timing for you. Just call this.update() periodically.
 */
class LEDBlinker {
public:
    /**
     * duration in ms.
     */
    LEDBlinker(int pin, int on_dur, int off_dur) {
        this->pin = pin;
        this->on_dur = on_dur;
        this->off_dur = off_dur;
        state = false;
        last_time = millis();

        pinMode(pin, OUTPUT);
    }

    void update() {
        int now = millis();
        int wait_time = (state ? on_dur : off_dur);
        if (now - last_time > wait_time) {
            state = !state;
            digitalWrite(pin, state);
            last_time = now;
        }
    }

private:
    int pin;
    int on_dur, off_dur;
    bool state;
    int last_time;
};
