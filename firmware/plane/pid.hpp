#pragma once

#include "utils.hpp"


class PIDControl {
public:
    PIDControl(float kp, float ki, float kd, float target, float integral_clamp = 10) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->target = target;
        this->integral_clamp = integral_clamp;
        integral = 0;
        last_time = micros();
        last_error = 0;
    }

    /**
     * Returns what to set motor/controller to.
     */
    float control(float value) {
        const uint32_t now = micros();
        float error = value - target;
        float deriv = (error - last_error) / (now - last_time) * 1e6;
        integral += error;
        integral = constrainf(integral, -integral_clamp, integral_clamp);

        last_error = error;
        last_time = micros();

        float ctrl = (
            kp * error
          + ki * integral
          + kd * deriv
        );
        return ctrl;
    }

private:
    float kp, ki, kd;
    float target;
    float integral_clamp;
    float integral;

    float last_error;
    // us. Rollover doesn't matter bc we are controlling often.
    uint32_t last_time;
};
