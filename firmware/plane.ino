#include <RH_ASK.h>
#include <ServoTimer2.h>

constexpr int PULSE_MIN = 700;
constexpr int PULSE_MAX = 2300;

RH_ASK _radio;

// right_motor, left_motor, right_aileron, left_aileron, right_elevator, left_elevator
ServoTimer2 _motors[6];
constexpr int _motor_pins[6] = {3, 4, 5, 6, 7, 8};
// standard (0, 1000), range unidirectional (0, 1000)
constexpr int _motor_limits[6][3] = {
    {0, 1000},
    {0, 1000},
    {600, 400},
    {400, -400},
    {500, 400},
    {500, 400},
};


/**
 * @param value  -1000 to 1000 interpolating between min and max
 */
void set_motor(int motor, int value) {
    int std = _motor_limits[motor][0];
    int range = _motor_limits[motor][1];
    int up = std + range;
    int down = std - range;
    int fac = map(value, -1000, 1000, down, up);
    int pulse = map(fac, 0, 1000, PULSE_MIN, PULSE_MAX);
    pulse = constrain(pulse, PULSE_MIN, PULSE_MAX);

    _motors[motor].write(pulse);
}


void test_servos() {
    for (int i = 2; i < 6; i++)
        set_motor(i, -1000);
    delay(1000);
    for (int i = 2; i < 6; i++)
        set_motor(i, 1000);
    delay(1000);
    for (int i = 2; i < 6; i++)
        set_motor(i, 0);
    delay(1000);
}

void test_motors() {
    set_motor(0, 150);
    set_motor(1, 150);
    delay(100);
    set_motor(0, 0);
    set_motor(1, 0);
    delay(1000);
}

void setup() {
    //Serial.begin(9600);

    if (!_radio.init())
        Serial.println("radio init failed");

    for (int i = 0; i < 5; i++) {
        _motors[i].attach(_motor_pins[i]);
        set_motor(i, 0);
    }

    delay(8000);
    test_servos();
    test_motors();

    //set_motor(0, 150);
    //set_motor(1, 150);
}


void loop() {
    /*
    uint8_t message[1];
    uint8_t len = sizeof(buf);
    if (_radio.recv(message, &len)) {
        int ailerons = message[0] - 128;
        int ail_r = map(ailerons, 0, 128, _motor_limits[2][2], _motor_limits[2][1]);
    }
    */
}
