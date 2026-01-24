#include <Servo.h>
#include "updateServos.h"

#define NUM_SERVOS 6
Servo s[NUM_SERVOS];

int crntAngles[6] = {90,90,90,90,90,90};
int prevAngles[6] = {90,90,90,90,90,90};

int servoPins[6] = {3,5,6,9,10,11};
////////////////////////
int activeServo = 1;
//////////////////////

int angles = 90;

float slowness = 20;

void setup() {
    Serial.begin(9600);
    s[0].attach(servoPins[0],500,2400);
    s[1].attach(servoPins[1],500,2330);
    s[2].attach(servoPins[2],725,2050);
    s[3].attach(servoPins[3],500,2500);
    s[4].attach(servoPins[4],500,1900);
    s[5].attach(servoPins[5],540,1800);
    for (int i = 0; i < NUM_SERVOS; i++) {
        s[i].write(prevAngles[i]);
    }

    delay(1000);
}

void loop() {
  if (Serial.available() > 0) {

    int servoInput = Serial.parseInt();   // human input: 1–6
    int angleInput = Serial.parseInt();   // 0–180

    // Convert to 0-based index
    servoInput = servoInput - 1;

    // ---- VALIDATION ----
    if (servoInput < 0 || servoInput >= NUM_SERVOS) {
      Serial.println("Invalid servo number");
      return;
    }

    angleInput = constrain(angleInput, 0, 180);

    // ---- APPLY ----
    crntAngles[servoInput] = angleInput;

    updateServos(prevAngles, crntAngles, slowness);

    prevAngles[servoInput] = crntAngles[servoInput];
  }
}
