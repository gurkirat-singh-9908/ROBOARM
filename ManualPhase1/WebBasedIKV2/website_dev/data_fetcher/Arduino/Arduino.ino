#include <Servo.h>
#include "updateServos.h"

Servo s1, s2, s3, s4, s5, s6, gripper;

int crntAngles[6] = {90, 90, 90, 90, 90, 90};

void setup() {
  Serial.begin(115200);

  // Attach servos with microsecond limits
  s1.attach(3, 500, 2400);   // Servo 1 (35kg)
  s2.attach(5, 500, 2300);   // Servo 2 (150kg)
  s3.attach(6, 725, 2050);   // Servo 3 (150kg)
  s4.attach(9, 500, 2400);   // Servo 4 (35kg)
  s5.attach(10, 575, 1900);  // Servo 5 (15kg)
  s6.attach(11, 575, 1900);  // Servo 6 (15kg)
  gripper.attach(12);        // Gripper — maps 0-100% to 0-180 deg

  Serial.println("Setup complete");
}

void loop() {

  if (Serial.available()) {

    String data = Serial.readStringUntil('\n');
    data.trim();

    int values[8];        // NEW packet size = 8
    int index = 0;

    char buffer[data.length() + 1];
    data.toCharArray(buffer, sizeof(buffer));

    char *token = strtok(buffer, " ");
    while (token != NULL && index < 8) {
      values[index++] = atoi(token);
      token = strtok(NULL, " ");
    }

    // Ensure full packet received
    if (index != 8) {
      Serial.println("Packet size error");
      return;
    }

    // Checksum validation
    int calcChecksum = 0;
    for (int i = 0; i < 7; i++) {
      calcChecksum += values[i];
    }

    if (calcChecksum != values[7]) {
      Serial.println("Checksum fail");
      return;
    }

    // Update servo target angles (s1-s6)
    for (int i = 0; i < 6; i++) {
      crntAngles[i] = constrain(values[i], 0, 180);
    }

    // Gripper: values[6] is 0-100 percent → map to 0-180 deg
    int gripperDeg = constrain(map(values[6], 0, 100, 0, 180), 0, 180);
    gripper.write(gripperDeg);

    updateServos(crntAngles);
    Serial.println("Packet OK");
  }
}
