#include <Servo.h>
#include "updateServos.h"

// Gripper DC motor driver pins (IN1 / IN2 on H-bridge).
// Pins 12/13 are not PWM-capable on Uno, so motor runs at full
// speed only — direction + run-time are the control variables.
#define GRIPPER_IN1 12
#define GRIPPER_IN2 13

// Safety cap on a single motor pulse.
// Calibrated full sweep is 15.0 s; effective sweep (derated for grip
// traction) is 13.5 s. Cap = 14000 ms gives a single packet enough room
// for a full 0→100 % sweep with 500 ms slack while still hard-braking if
// host sends something pathological.
static const unsigned long MAX_PULSE_MS = 16000;

Servo s1, s2, s3, s4, s5, s6;

int crntAngles[6] = {90, 90, 90, 90, 90, 90};

// Gripper non-blocking pulse state.
//   gripperDir:   -1 = close, 0 = idle/brake, +1 = open
//   gripperEndMs: millis() value at which the active pulse ends
int8_t gripperDir = 0;
unsigned long gripperEndMs = 0;

// E-stop. true = frozen: gripper braked, motion packets ignored.
// Engaged/released by the out-of-band control bytes "X"/"G" (sent by
// bridge_node on /roboarm/estop).
bool estopped = false;

void setGripper(int8_t dir) {
  // Both LOW is coast on L298N / brake on TB6612 — safe default.
  // Switch to (HIGH,HIGH) for active brake if the driver coasts open.
  if (dir > 0) {
    digitalWrite(GRIPPER_IN1, HIGH);
    digitalWrite(GRIPPER_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(GRIPPER_IN1, LOW);
    digitalWrite(GRIPPER_IN2, HIGH);
  } else {
    digitalWrite(GRIPPER_IN1, LOW);
    digitalWrite(GRIPPER_IN2, LOW);
  }
  gripperDir = (dir > 0) ? 1 : (dir < 0 ? -1 : 0);
}

void startGripperPulse(int signedMs) {
  if (signedMs == 0) {
    // 0 = "no gripper command in this packet" — do NOT brake here, or the
    // idle packets streaming between commands would kill every pulse after
    // one serial interval. tickGripper() brakes at the pulse's timed end.
    return;
  }
  unsigned long mag = (unsigned long) (signedMs < 0 ? -signedMs : signedMs);
  if (mag > MAX_PULSE_MS) mag = MAX_PULSE_MS;
  setGripper(signedMs > 0 ? 1 : -1);
  gripperEndMs = millis() + mag;
}

void tickGripper() {
  if (gripperDir != 0 && (long)(millis() - gripperEndMs) >= 0) {
    setGripper(0);
  }
}

void engageEstop() {
  estopped = true;
  setGripper(0);             // brake the motor NOW, even mid-pulse
  gripperEndMs = millis();   // disarm the pulse timer
  Serial.println("ESTOP");
}

void releaseEstop() {
  estopped = false;
  Serial.println("RESUME");
}

void setup() {
  Serial.begin(115200);

  // Attach servos with microsecond limits
  s1.attach(3, 500, 2400);   // Servo 1 (35kg)
  s2.attach(5, 500, 2300);   // Servo 2 (150kg)
  s3.attach(6, 725, 2050);   // Servo 3 (150kg)
  s4.attach(9, 500, 2400);   // Servo 4 (35kg)
  s5.attach(10, 575, 1900);  // Servo 5 (15kg)
  s6.attach(11, 575, 1900);  // Servo 6 (15kg)

  // Gripper DC motor driver.
  pinMode(GRIPPER_IN1, OUTPUT);
  pinMode(GRIPPER_IN2, OUTPUT);
  setGripper(0);

  Serial.println("Setup complete");
}

void loop() {
  // Brake the gripper as soon as the pulse window elapses, regardless of
  // whether a new packet is available — keeps motor timing independent of
  // serial cadence.
  tickGripper();

  if (Serial.available()) {

    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length() == 0) return;

    // Out-of-band control bytes (checked before packet parse).
    if (data == "X" || data == "x") { engageEstop();  return; }
    if (data == "G" || data == "g") { releaseEstop(); return; }

    // While frozen, drop motion packets (gripper stays braked, servos hold).
    if (estopped) return;

    int values[8];        // packet: s1..s6, gripper_pulse_ms (signed), checksum
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

    // Gripper: values[6] is signed pulse-ms.
    //   > 0 → open for that many ms
    //   < 0 → close for that many ms
    //   = 0 → no motion (brake)
    startGripperPulse(values[6]);

    updateServos(crntAngles);
    Serial.println("Packet OK");
  }
}
