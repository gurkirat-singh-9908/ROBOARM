#include <Servo.h>
#include "updateServos.h"

Servo s1, s2, s3, s4, s5, s6;

int crntAngles[6] = {90, 90, 90, 90, 90, 90};
int prevAngles[6] = {90, 90, 90, 90, 90, 90};

int totalDelay = 500;  // Total transition time (ms)
int mul = 10;          // Multiplier for slowness
float slowness = totalDelay * mul / 100.0;  // 50ms per step

void setup() {
  Serial.begin(9600);
  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  s4.attach(9);
  s5.attach(10);
  s6.attach(11);

  Serial.println("Setup complete");
  s1.write(prevAngles[0]);
  s2.write(prevAngles[1]);
  s3.write(prevAngles[2]);
  s4.write(prevAngles[3]);
  s5.write(prevAngles[4]);
  s6.write(prevAngles[5]);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Parse the string
    int firstSpace = data.indexOf(' ');
    int secondSpace = data.indexOf(' ', firstSpace + 1);
    
    if (firstSpace == -1 || secondSpace == -1) {
      Serial.println("Invalid format");
      return;
    }

    // Extract values
    int value1 = data.substring(0, firstSpace).toInt();
    int value2 = data.substring(firstSpace + 1, secondSpace).toInt();
    int checksum = data.substring(secondSpace + 1).toInt();

    // Validate servo ID and angle
    if (value1 < 1 || value1 > 6) {
      Serial.println(String(value1) + " " + String(value2) + " F: Invalid servo ID");
      return;
    }
    if (value2 < 0 || value2 > 180) {
      Serial.println(String(value1) + " " + String(value2) + " F: Invalid angle");
      return;
    }

    // Check checksum
    if (checksum == value1 + value2) {
      Serial.println(String(value1) + " " + String(value2) + " T");
      crntAngles[value1 - 1] = value2;
      updateServos(prevAngles, crntAngles, slowness);
      // Copy crntAngles to prevAngles
      for (int i = 0; i < 6; i++) {
        prevAngles[i] = crntAngles[i];
      }
    } else {
      Serial.println(String(value1) + " " + String(value2) + " F: Checksum fail");
    }
  }
}
