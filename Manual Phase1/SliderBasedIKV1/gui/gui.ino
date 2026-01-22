#include <Servo.h>
#include "updateServos.h"

String inputString = "";
bool stringComplete = false;

void setup() {
    Serial.begin(9600);
    initServos();
}

void loop() {
    serialEvent();
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }

    if (stringComplete) {
        parseCommand(inputString);
        inputString = "";
        stringComplete = false;
    }
}

void parseCommand(String cmd) {
    // Expected format: servo angle speed
    int s, angle;
    float speed;

    sscanf(cmd.c_str(), "%d %d %f", &s, &angle, &speed);

    moveServoSmooth(s, angle, speed);
}
