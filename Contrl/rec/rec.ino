/*
 * rec.ino  –  Arduino servo + gripper receiver for ROBOARM
 *
 * Motion protocol (sent by bridge_node.py):
 *   "<j0> <j1> <j2> <j3> <j4> <j5> <gripper> <checksum>\n"
 *
 *   8 space-separated integers followed by newline.
 *   joints  j0..j5 : servo angles, 0–180.
 *   gripper        : SIGNED DC-motor pulse in ms (not an angle!).
 *                      > 0  run "open" direction for that many ms
 *                      < 0  run "close" direction for that many ms
 *                      = 0  no gripper command this packet (motor untouched)
 *   checksum = sum of the first 7 values (modulo 65536).
 *   Bad checksum / parse → command discarded.
 *
 * Control bytes (single char + newline, sent out-of-band by the bridge):
 *   "X\n"  E-STOP  — brake the gripper motor IMMEDIATELY (even mid-pulse)
 *                    and freeze: ignore all motion packets until released.
 *   "G\n"  GO      — release the e-stop and resume accepting motion.
 *
 * Baud rate: 115200  (must match bridge_node.py baud_rate parameter)
 *
 * Servo pulse calibration (from read.me.txt):
 *   s1 (35 kg)  : 500 – 2400 µs    s4 (35 kg)  : 500 – 2400 µs
 *   s2 (150 kg) : 500 – 2300 µs    s5 (15 kg)  : 575 – 1900 µs
 *   s3 (150 kg) : 725 – 2050 µs    s6 (15 kg)  : 575 – 1900 µs
 *
 * Gripper: DC motor via H-bridge on IN1=pin 12, IN2=pin 13 (no PWM → full
 * speed; direction + run-time are the only controls). Servos use pins 2–7,
 * so 12/13 are free.
 */

#include <Servo.h>

// ── Servo objects ────────────────────────────────────────────────────────────
Servo s1, s2, s3, s4, s5, s6;

// ── Calibrated pulse-width ranges (µs) per servo ────────────────────────────
const int S_MIN_US[6] = { 500, 500, 725, 500, 575, 575 };
const int S_MAX_US[6] = { 2400, 2300, 2050, 2400, 1900, 1900 };

// ── Servo pin assignments ────────────────────────────────────────────────────
const int SERVO_PINS[6] = { 2, 3, 4, 5, 6, 7 };

// ── Gripper DC motor (H-bridge) ──────────────────────────────────────────────
#define GRIPPER_IN1 12
#define GRIPPER_IN2 13
// Hard cap on a single pulse so a pathological host value can't run the motor
// forever. Full sweep ≈ 15 s; 16000 ms leaves room for a full 0→100 %.
static const unsigned long MAX_PULSE_MS = 16000;

// ── Home position (degrees) ──────────────────────────────────────────────────
const int HOME_DEG = 90;

// ── Internal state ───────────────────────────────────────────────────────────
int joint[6] = { 0 };            // [0..5] = servo angles

// Gripper non-blocking pulse state.
//   gripperDir:   -1 = close, 0 = idle/brake, +1 = open
//   gripperEndMs: millis() value at which the active pulse ends
int8_t gripperDir = 0;
unsigned long gripperEndMs = 0;

bool estopped = false;           // true = frozen: brake gripper, ignore motion

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

// ── Gripper motor control ────────────────────────────────────────────────────
void setGripper(int8_t dir) {
  // Both LOW = coast on L298N / brake on TB6612 — safe default.
  if (dir > 0) {                 // open
    digitalWrite(GRIPPER_IN1, HIGH);
    digitalWrite(GRIPPER_IN2, LOW);
  } else if (dir < 0) {          // close
    digitalWrite(GRIPPER_IN1, LOW);
    digitalWrite(GRIPPER_IN2, HIGH);
  } else {                       // brake / idle
    digitalWrite(GRIPPER_IN1, LOW);
    digitalWrite(GRIPPER_IN2, LOW);
  }
  gripperDir = (dir > 0) ? 1 : (dir < 0 ? -1 : 0);
}

void startGripperPulse(int signedMs) {
  if (signedMs == 0) return;     // no command this packet; timed brake handles end
  unsigned long mag = (unsigned long)(signedMs < 0 ? -signedMs : signedMs);
  if (mag > MAX_PULSE_MS) mag = MAX_PULSE_MS;
  setGripper(signedMs > 0 ? 1 : -1);
  gripperEndMs = millis() + mag;
}

void tickGripper() {
  // Brake when the pulse window elapses, independent of serial cadence.
  if (gripperDir != 0 && (long)(millis() - gripperEndMs) >= 0) {
    setGripper(0);
  }
}

// ── E-stop ───────────────────────────────────────────────────────────────────
void engageEstop() {
  estopped = true;
  setGripper(0);                 // brake the motor NOW, even mid-pulse
  gripperEndMs = millis();       // disarm the pulse timer
  Serial.println("ESTOP");
}

void releaseEstop() {
  estopped = false;
  Serial.println("RESUME");
}

void setup() {
  Serial.begin(115200);          // must match bridge_node.py baud_rate

  Servo *servos[6] = { &s1, &s2, &s3, &s4, &s5, &s6 };
  for (int i = 0; i < 6; i++) {
    servos[i]->attach(SERVO_PINS[i]);
    writeServo(*servos[i], i, HOME_DEG);
  }

  pinMode(GRIPPER_IN1, OUTPUT);
  pinMode(GRIPPER_IN2, OUTPUT);
  setGripper(0);

  Serial.println("ROBOARM ready – waiting for commands at 115200 baud");
}

void loop() {
  // Always service the gripper timer first so motor timing is independent of
  // serial cadence (and so a pulse still brakes itself if no packet arrives).
  tickGripper();

  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // ── Out-of-band control bytes (checked before packet parse) ─────────────
    if (line == "X" || line == "x") { engageEstop();  return; }
    if (line == "G" || line == "g") { releaseEstop(); return; }

    // While frozen, drop motion packets (gripper stays braked, servos hold).
    if (estopped) return;

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

    // ── Validate checksum (sum of first 7 values) ───────────────────────────
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

    // ── Store joint angles (0-180). Gripper (values[6]) stays signed. ───────
    for (int i = 0; i < 6; i++) joint[i] = constrain(values[i], 0, 180);

    // ── Drive servos (physical wiring order kept from original mapping) ──────
    // s1(pin2)=j1, s2(pin3)=j2, s3(pin4)=j3, s4(pin5)=j4,
    // s5(pin6)=j5, s6(pin7)=j0   (circular shift – matches physical wiring)
    writeServo(s1, 0, joint[1]);
    writeServo(s2, 1, joint[2]);
    writeServo(s3, 2, joint[3]);
    writeServo(s4, 3, joint[4]);
    writeServo(s5, 4, joint[5]);
    writeServo(s6, 5, joint[0]);

    // ── Gripper: signed pulse-ms (>0 open, <0 close, 0 = no change) ──────────
    startGripperPulse(values[6]);

    // ── Echo back for bridge_node.py feedback logging ────────────────────────
    Serial.print("OK:");
    for (int i = 0; i < 6; i++) {
      Serial.print(joint[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.print(" grip=");
    Serial.println(values[6]);
  }
}
