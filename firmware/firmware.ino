#include <RH_ASK.h>

using ull = unsigned long long;


/**
 * Time in us.
 */
ull time() {
    return 1000*millis() + micros()%1000;
}


/**
 * Servo motor that doesn't need the hardware clock thing.
 * After doing motor.write(angle), call motor.update() very frequently so
 * the pulse is correct.
 */
class ServoMotor {
private:
    int pin;
    // us
    int pulse_width;

public:
    const int min_width = 700;
    const int max_width = 2500;

    ServoMotor() {
    }

    void attach(int pin) {
        this->pin = pin;
    }

    void write(int angle) {
        pulse_width = map(angle, 0, 180, min_width, max_width);
        pulse_width = constrain(pulse_width, min_width, max_width);
    }

    void update() {
        /*
        const int t = time() % period;
        if (t < pulse_width) {
            digitalWrite(pin, HIGH);
        } else {
            digitalWrite(pin, LOW);
        }
        */
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse_width);
        digitalWrite(pin, LOW);
    }
};


// motors, right_aileron, left_aileron, right_elevator, left_elevator
ServoMotor _motors[6];
constexpr int _motor_pins[6] = {3, 4, 5, 6, 7, 8};
// min (0, 180), max (0, 180), standard (0, 1000)
constexpr int _motor_limits[6][3] = {
    {50, 180, 0},
    {50, 180, 0},
    {180, 50, 300},
    {20, 160, 300},
    {0, 180, 500},
    {180, 0, 500}
};


/**
 * Call update on all motors for specified time.
 */
void update_and_delay(int ms) {
    const int start = millis();
    while (millis() - start < ms) {
        for (int i = 0; i < 6; i++) {
            _motors[i].update();
        }
        delay(19);
    }
}


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
    update_and_delay(1000);
    for (int i = 2; i < 6; i++)
        set_motor(i, 1000);
    update_and_delay(1000);
    for (int i = 2; i < 6; i++)
        set_motor(i, _motor_limits[i][2]);
    update_and_delay(1000);
}

void test_motors() {
    set_motor(0, 100);
    set_motor(1, 100);
    update_and_delay(100);
    set_motor(0, 0);
    set_motor(1, 0);
    update_and_delay(1000);
}

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < 5; i++) {
        _motors[i].attach(_motor_pins[i]);
        set_motor(i, _motor_limits[i][2]);
    }

    set_motor(0, 1000);
    set_motor(1, 1000);
    set_motor(2, 1000);
    update_and_delay(6000);
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    update_and_delay(6000);
    return;

    delay(8000);
    test_servos();
    test_motors();
}


void loop() {
}
