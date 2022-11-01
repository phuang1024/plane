#pragma once

#include "ServoTimer2.h"


class CtrlSurface {
private:
    ServoTimer2* servo;
    int zero;

public:
    /**
     * @param zero  Pulse width (ms) of zero position.
     */
    CtrlSurface(ServoTimer2& servo, int pin, int zero) {
        this->servo = &servo;
        this->servo->attach(pin);
        this->zero = zero;
    }

    /**
     * servo.write(zero+delta);
     */
    void write(int delta) {
        servo->write(zero + delta);
    }
};
