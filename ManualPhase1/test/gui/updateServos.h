#ifndef UPDATE_SERVOS_H
#define UPDATE_SERVOS_H

#include <Servo.h>

#define NUM_SERVOS 6

extern Servo s[NUM_SERVOS];

void updateServos(int prev[], int curr[], float slowness);

#endif
