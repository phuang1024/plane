#include <RH_ASK.h>
#include <ServoTimer2.h>

using ull = unsigned long long;

constexpr int PULSE_MIN = 700;
constexpr int PULSE_MAX = 2300;

RH_ASK _radio;

// motors, right_aileron, left_aileron, elevators
ServoTimer2 _motors[6];
constexpr int _motor_pins[6] = {3, 4, 5, 6};
// standard (0, 1000), range from std to min/max (0, 1000)
constexpr int _motor_limits[6][3] = {
    {0, 1000},
    {600, 500},
    {400, -500},
    {650, 250},
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
    for (int i = 1; i < 4; i++)
        set_motor(i, -1000);
    delay(1000);
    for (int i = 1; i < 4; i++)
        set_motor(i, 1000);
    delay(1000);
    for (int i = 1; i < 4; i++)
        set_motor(i, 0);
    delay(1000);
}

void test_motors() {
    set_motor(0, 150);
    delay(100);
    set_motor(0, 0);
    delay(1000);
}

void setup() {
    Serial.begin(9600);

    if (!_radio.init())
        Serial.println("radio init failed");

    for (int i = 0; i < 4; i++) {
        _motors[i].attach(_motor_pins[i]);
        set_motor(i, 0);
    }

    delay(8000);
    test_servos();
    test_motors();

    //set_motor(0, 150);

    // Loop
    ull last_recv = millis();
    while (true) {
        // Process radio message. See rc.ino for protocol.
        uint8_t message[3];
        uint8_t len = sizeof(message);
        if (_radio.recv(message, &len)) {
            int ailerons = message[0] - 128;
            int ail_r = map(ailerons, -128, 127, 1000, -1000);
            int ail_l = map(ailerons, -128, 127, -1000, 1000);
            set_motor(1, ail_r);
            set_motor(2, ail_l);

            int elevator = message[1] - 128;
            int elev_ctrl = map(elevator, -128, 127, -1000, 1000);
            set_motor(3, elev_ctrl);

            bool motors_on = message[2];
            Serial.println(motors_on);
            Serial.println(message[2]);
            set_motor(0, motors_on ? 150 : 0);

            last_recv = millis();
        }

        // Turn off motors if no message received for 1 second
        ull now = millis();
        if (now - last_recv > 1000) {
            set_motor(0, 0);
        }
    }
}

void loop() {
    // Loop is in setup().
}
