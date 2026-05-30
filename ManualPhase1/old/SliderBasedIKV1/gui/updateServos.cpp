#include <Arduino.h>
#include "updateServos.h"

Servo servos[NUM_SERVOS];
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};

void initServos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].write(90);
    }
}

void moveServoSmooth(int servoNum, int targetAngle, int slowness) {

    if (servoNum < 0 || servoNum >= NUM_SERVOS) return;

    int current = currentAngles[servoNum];

    if (current < targetAngle) {
        for (int a = current; a <= targetAngle; a++) {
            servos[servoNum].write(a);
            delay(slowness);
        }
    } else {
        for (int a = current; a >= targetAngle; a--) {
            servos[servoNum].write(a);
            delay(slowness);
        }
    }

    currentAngles[servoNum] = targetAngle;
}
