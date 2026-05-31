/*
 * Arduino_GripperCalibrate.ino
 *
 * Standalone calibration sketch for the DC-motor gripper. NOT the production
 * firmware — flash this only while calibrating, then flash the regular
 * Arduino.ino back.
 *
 * Servos are hard-wired to 90° in setup() and never touched again, so the
 * arm sits still while you focus on the gripper.
 *
 * Open Arduino IDE Serial Monitor at 115200 baud, line ending = Newline.
 * Send single-character commands (Enter sends them):
 *
 *   f  close gripper  (direction A — which one depends on wiring)
 *   b  open gripper   (direction B)
 *   p  pause          (brake motor + pause timer, print elapsed)
 *   s  start          (arm timer, zero accumulator)
 *   r  reset          (brake motor, disarm timer, zero accumulator)
 *   ?  print state
 *
 * Wire-flip safe: the timer counts whenever the motor is running and
 * recording is armed, no matter which letter started it. Typical session:
 *
 *   b  -> open gripper to one mechanical limit
 *   p  -> stop at the limit
 *   s  -> arm timer
 *   f  -> close gripper to the opposite limit
 *   p  -> stop; sketch prints total milliseconds and ms_per_percent
 *
 * Safety: if no command arrives for SAFETY_TIMEOUT_MS while motor is on,
 * the motor brakes automatically. Adjust below if your full sweep is
 * longer.
 */

#include <Servo.h>

#define GRIPPER_IN1 12
#define GRIPPER_IN2 13

// Brake motor if no command received within this window while running.
// Keep it comfortably longer than your expected full sweep.
static const unsigned long SAFETY_TIMEOUT_MS = 15000;

Servo s1, s2, s3, s4, s5, s6;

int8_t        gripperDir          = 0;       // -1, 0, +1
unsigned long motorStartMs        = 0;       // when current run began
unsigned long lastCommandMs       = 0;       // for safety timeout
bool          recording           = false;
unsigned long accumulatedMs       = 0;

void setGripper(int8_t dir) {
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

void startMotor(int8_t dir, const char *label) {
  if (gripperDir == dir) {
    Serial.print(label);
    Serial.println(" (already running)");
    return;
  }

  // Direction change while running counts toward the timer up to this point.
  if (recording && gripperDir != 0) {
    accumulatedMs += millis() - motorStartMs;
  }

  setGripper(dir);
  motorStartMs  = millis();
  lastCommandMs = motorStartMs;

  Serial.print(label);
  if (recording) Serial.print(" - timer running");
  Serial.println();
}

void pauseMotor(bool verbose) {
  if (gripperDir == 0) {
    if (verbose) Serial.println("paused (motor already idle)");
    return;
  }

  if (recording) {
    accumulatedMs += millis() - motorStartMs;
  }
  setGripper(0);

  if (verbose) {
    if (recording) {
      Serial.print("paused. total_ms=");
      Serial.print(accumulatedMs);
      Serial.print("  ms_per_percent=");
      Serial.println(accumulatedMs / 100.0, 2);
    } else {
      Serial.println("paused");
    }
  }
}

void resetAll() {
  pauseMotor(false);
  recording     = false;
  accumulatedMs = 0;
  motorStartMs  = 0;
  Serial.println("reset (disarmed, total_ms=0)");
}

void armRecording() {
  pauseMotor(false);
  recording     = true;
  accumulatedMs = 0;
  motorStartMs  = 0;
  Serial.println("recording armed (total_ms=0)");
}

void printState() {
  Serial.print("dir=");
  Serial.print((int)gripperDir);
  Serial.print("  recording=");
  Serial.print(recording ? "yes" : "no");
  Serial.print("  total_ms=");
  unsigned long live = accumulatedMs;
  if (recording && gripperDir != 0) live += millis() - motorStartMs;
  Serial.println(live);
}

void handleChar(char c) {
  switch (c) {
    case 'f': case 'F': startMotor(+1, "close");    lastCommandMs = millis(); break;
    case 'b': case 'B': startMotor(-1, "open");     lastCommandMs = millis(); break;
    case 'p': case 'P': pauseMotor(true);           lastCommandMs = millis(); break;
    case 's': case 'S': armRecording();             lastCommandMs = millis(); break;
    case 'r': case 'R': resetAll();                 lastCommandMs = millis(); break;
    case '?':           printState();               break;
    case '\r': case '\n': case ' ': case '\t': break;
    default:
      Serial.print("unknown: ");
      Serial.println(c);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Servos parked at 90 deg for the entire calibration session.
  s1.attach(3,  500, 2400);
  s2.attach(5,  500, 2300);
  s3.attach(6,  725, 2050);
  s4.attach(9,  500, 2400);
  s5.attach(10, 575, 1900);
  s6.attach(11, 575, 1900);
  s1.write(90); s2.write(90); s3.write(90);
  s4.write(90); s5.write(90); s6.write(90);

  pinMode(GRIPPER_IN1, OUTPUT);
  pinMode(GRIPPER_IN2, OUTPUT);
  setGripper(0);

  lastCommandMs = millis();

  Serial.println();
  Serial.println("=== Gripper calibration ===");
  Serial.println("cmds: f=close  b=open  p=pause  s=start  r=reset  ?=state");
}

void loop() {
  while (Serial.available()) {
    handleChar((char) Serial.read());
  }

  // Safety: kill motor if user goes silent while it's running.
  if (gripperDir != 0 && (millis() - lastCommandMs) > SAFETY_TIMEOUT_MS) {
    Serial.println("SAFETY TIMEOUT - braking motor");
    pauseMotor(true);
  }
}
