/*
 * rec.ino  –  Arduino servo receiver for ROBOARM
 *
 * Protocol (sent by bridge_node.py):
 *   "<j0> <j1> <j2> <j3> <j4> <j5> <gripper> <checksum>\n"
 *
 *   8 space-separated integers followed by newline.
 *   checksum = sum of the first 7 values (modulo 65536).
 *   If checksum does not match, the command is silently discarded.
 *
 * Baud rate: 115200  (must match bridge_node.py baud_rate parameter)
 *
 * Servo pulse calibration (from read.me.txt):
 *   s1 (35 kg)  : 500 – 2400 µs
 *   s2 (150 kg) : 500 – 2300 µs
 *   s3 (150 kg) : 725 – 2050 µs
 *   s4 (35 kg)  : 500 – 2400 µs
 *   s5 (15 kg)  : 575 – 1900 µs
 *   s6 (15 kg)  : 575 – 1900 µs
 */

#include <Servo.h>

// ── Servo objects ────────────────────────────────────────────────────────────
Servo s1, s2, s3, s4, s5, s6;

// ── Calibrated pulse-width ranges (µs) per servo ────────────────────────────
const int S_MIN_US[6] = { 500, 500, 725, 500, 575, 575 };
const int S_MAX_US[6] = { 2400, 2300, 2050, 2400, 1900, 1900 };

// ── Servo pin assignments ────────────────────────────────────────────────────
const int SERVO_PINS[6] = { 2, 3, 4, 5, 6, 7 };

// ── Home position (degrees) ──────────────────────────────────────────────────
const int HOME_DEG = 90;

// ── Internal state ───────────────────────────────────────────────────────────
int joint[7] = { 0 };   // [0..5] = joints, [6] = gripper

// ── Helper: convert degree (0-180) → calibrated pulse width (µs) ────────────
int degToUs(int servoIdx, int deg) {
  deg = constrain(deg, 0, 180);
  return S_MIN_US[servoIdx] +
         (int)((long)deg * (S_MAX_US[servoIdx] - S_MIN_US[servoIdx]) / 180);
}

// ── Helper: write degree to a servo using calibrated µs ──────────────────────
void writeServo(Servo &srv, int servoIdx, int deg) {
  srv.writeMicroseconds(degToUs(servoIdx, deg));
}

void setup() {
  Serial.begin(115200);   // must match bridge_node.py baud_rate

  Servo *servos[6] = { &s1, &s2, &s3, &s4, &s5, &s6 };
  for (int i = 0; i < 6; i++) {
    servos[i]->attach(SERVO_PINS[i]);
    writeServo(*servos[i], i, HOME_DEG);
  }

  Serial.println("ROBOARM ready – waiting for commands at 115200 baud");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // ── Parse 8 integers ────────────────────────────────────────────────────
    int values[8] = { 0 };
    int parsed = sscanf(line.c_str(),
                        "%d %d %d %d %d %d %d %d",
                        &values[0], &values[1], &values[2], &values[3],
                        &values[4], &values[5], &values[6], &values[7]);

    if (parsed < 8) {
      Serial.print("ERR:bad_parse(got ");
      Serial.print(parsed);
      Serial.println(")");
      return;
    }

    // ── Validate checksum ────────────────────────────────────────────────────
    int computedSum = 0;
    for (int i = 0; i < 7; i++) computedSum += values[i];
    computedSum &= 0xFFFF;

    if (computedSum != values[7]) {
      Serial.print("ERR:checksum_fail(rx=");
      Serial.print(values[7]);
      Serial.print(",calc=");
      Serial.print(computedSum);
      Serial.println(")");
      return;
    }

    // ── Store joint values ───────────────────────────────────────────────────
    for (int i = 0; i < 7; i++) joint[i] = constrain(values[i], 0, 180);

    // ── Drive servos (physical wiring order kept from original mapping) ──────
    // s1(pin2)=j1, s2(pin3)=j2, s3(pin4)=j3, s4(pin5)=j4,
    // s5(pin6)=j5, s6(pin7)=j0   (circular shift – matches physical wiring)
    writeServo(s1, 0, joint[1]);
    writeServo(s2, 1, joint[2]);
    writeServo(s3, 2, joint[3]);
    writeServo(s4, 3, joint[4]);
    writeServo(s5, 4, joint[5]);
    writeServo(s6, 5, joint[0]);

    // ── Echo back for bridge_node.py feedback logging ────────────────────────
    Serial.print("OK:");
    for (int i = 0; i < 6; i++) {
      Serial.print(joint[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.print(" grip=");
    Serial.println(joint[6]);
  }
}
