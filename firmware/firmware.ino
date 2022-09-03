#include <ServoTimer2.h>
#include <RH_ASK.h>


/*
class ServoMotor {
private:
    int pin;

public:
    // Microseconds
    const int pwm_period = 1e6 / 244.14;

    ServoMotor() {
    }

    void attach(int pin) {
        this->pin = pin;
        setPwmFrequency(pin, (pin == 5 || pin == 6) ? 256 : 128);
    }

    void write(int angle) {
        int target_width = map(angle, 0, 180, 1000, 2000);
        int duty = map(target_width, 0, pwm_period, 0, 255);
        analogWrite(pin, duty);
    }
};
*/


// motors, right_aileron, left_aileron, right_elevator, left_elevator
ServoTimer2 _motors[5];
constexpr int _motor_pins[5] = {3, 5, 6, 9, 10};
// min (0, 180), max (0, 180), standard (0, 1000)
constexpr int _motor_limits[5][3] = {
    {30, 180, 0},
    {180, 50, 400},
    {20, 160, 400},
    {0, 180, 500},
    {180, 0, 500}
};


/**
 * @param value  0 to 1000 interpolating between min and max
 */
void set_motor(int motor, int value) {
    int down = _motor_limits[motor][0];
    int up = _motor_limits[motor][1];
    int degree = map(value, 0, 1000, down, up);
    int width = map(degree, 0, 180, 1000, 2000);
    width = constrain(width, 1000, 2000);
    _motors[motor].write(width);
}


void test_servos() {
    for (int i = 1; i < 5; i++)
        set_motor(i, 0);
    delay(400);
    for (int i = 1; i < 5; i++)
        set_motor(i, 1000);
    delay(400);
    for (int i = 1; i < 5; i++)
        set_motor(i, _motor_limits[i][2]);
    delay(500);
}

void test_motors() {
    set_motor(0, 100);
    delay(100);
    set_motor(0, 0);
    delay(500);
}

void setup() {
    for (int i = 0; i < 5; i++) {
        _motors[i].attach(_motor_pins[i]);
        set_motor(i, _motor_limits[i][2]);
    }

    delay(8000);
    test_servos();
    test_motors();

    delay(1000);
}


void loop() {
    for (int i = 1000; i < 2500; i += 20) {
        _motors[1].write(i);
        delay(50);
    }
}
