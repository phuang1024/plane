#include <Servo.h>

// right_motor, left_motor, right_aileron, left_aileron, right_elevator, left_elevator
Servo _motors[6];
constexpr int _motor_pins[6] = {3, 5, 6, 9, 10, 11};
// min (0, 180), max (0, 180), standard (0, 1000)
constexpr int _motor_limits[6][3] = {
    {30, 180, 0},
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
    int v = map(value, 0, 1000, down, up);

    int min, max;
    if (down < up) {
        min = down;
        max = up;
    } else {
        min = up;
        max = down;
    }
    v = constrain(v, min, max);

    _motors[motor].write(v);
}


void test_servos() {
    for (int i = 2; i < 6; i++)
        set_motor(i, 0);
    delay(500);
    for (int i = 2; i < 6; i++)
        set_motor(i, 1000);
    delay(500);
    for (int i = 2; i < 6; i++)
        set_motor(i, _motor_limits[i][2]);
    delay(500);
}

void test_motors() {
    set_motor(0, 100);
    set_motor(1, 100);
    delay(100);
    set_motor(0, 0);
    set_motor(1, 0);
    delay(500);
}

void setup() {
    for (int i = 0; i < 6; i++) {
        _motors[i].attach(_motor_pins[i]);
        set_motor(i, _motor_limits[i][2]);
    }

    delay(8000);
    test_servos();
    test_motors();
}


void loop() {
}
