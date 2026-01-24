#ifndef UPDATE_SERVOS_H
#define UPDATE_SERVOS_H

#include <Servo.h>

#define NUM_SERVOS 6

extern Servo servos[NUM_SERVOS];
extern int currentAngles[NUM_SERVOS];

void initServos();
void moveServoSmooth(int servoNum, int targetAngle, int slowness);

#endif
