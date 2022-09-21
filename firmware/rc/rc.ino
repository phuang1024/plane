/*
Message protocol:
- The remote transmits the joystick positions only: It says "knob at 0" instead of "set motor to 0".
  This provides some security.
- Each message is 3 bytes: joystick x, joystick y, joystick button.
- Each analog signal is 0 to 255.
  - If negative values are needed, 127 means "at rest", 0 means "full negative",
    255 means "full positive".
  - Otherwise, 0 means "at rest" and 255 means "full positive".
- Each digital signal is 0 or non-zero.
*/

#include <RH_ASK.h>
#include <RotaryEncoder.h>

using ull = unsigned long long;

RH_ASK _radio(2000);
RotaryEncoder rotary(4, 5, RotaryEncoder::LatchMode::TWO03);

void setup() {
    Serial.begin(9600);
    if (!_radio.init())
         Serial.println("radio init failed");

    pinMode(6, OUTPUT);  // LED

    analogWrite(6, 5);

    // Loop
    ull last_send = 0;
    while (true) {
        delay(50);

        last_send = millis();

        int x = analogRead(0);
        int y = analogRead(1);
        int button = analogRead(2) == 0;

        unsigned char msg[3];
        msg[0] = map(x, 0, 1023, 0, 255);
        msg[1] = map(y, 0, 1023, 0, 255);
        msg[2] = button;
        _radio.send((uint8_t*)msg, sizeof(msg));
        _radio.waitPacketSent();
    }
}


void loop() {
}
