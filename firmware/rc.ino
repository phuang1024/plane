#include <RH_ASK.h>

RH_ASK _radio;

void setup() {
    if (!_radio.init())
         Serial.println("radio init failed");
}

void loop() {
    delay(50);

    int x = analogRead(0);

    unsigned char msg[1];
    msg[0] = map(x, 0, 1023, 0, 255);
    _radio.send((uint8_t*)msg, strlen(msg));
    _radio.waitPacketSent();
}
