#ifndef UPDATE_SERVOS_H
#define UPDATE_SERVOS_H

#include <Arduino.h>
#include <Servo.h>

extern Servo s1, s2, s3, s4, s5, s6;

void updateServos(int prevAngles[], int crntAngles[], float slowness);
void Write(int sno, int prevA, int crntA, float slowness);

#endif