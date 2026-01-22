#include <Servo.h> // Include the Servo library

// Create Servo objects
Servo servo1, servo2, servo3, servo4, servo5, servo6;

// Variables to track current angles
int currentAngles[6] = {90, 90, 90, 90, 90, 90};

void setup() {
  Serial.begin(9600); // Initialize Serial communication at 9600 baud
  
  // Attach servos to respective pins
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);
  servo6.attach(11);

  // Set all servos to default position 90°
  for (int i = 0; i < 6; i++) {
    moveServo(i+1, 90);
  }

  Serial.println("Robotic Arm Control System Initialized");
  Serial.println("Ready to receive commands");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    
    // Parse the input
    int spaceIdx = input.indexOf(' ');
    if (spaceIdx == -1) {
      Serial.println("Invalid command format. Use: '<servo_number> <angle>'");
      return;
    }
    
    int servoNumber = input.substring(0, spaceIdx).toInt();
    int angle = input.substring(spaceIdx + 1).toInt();
    
    // Check for valid input
    if (servoNumber >= 1 && servoNumber <= 6 && angle >= 0 && angle <= 180) {
      moveServo(servoNumber, angle);
    } else {
      Serial.println("Invalid input! Servo number: 1-6, Angle: 0-180");
    }
  }
}

// Function to move a servo smoothly
void moveServo(int servoNumber, int targetAngle) {
  // Get current angle
  int currentAngle = currentAngles[servoNumber-1];
  
  // Only move if the angle has changed significantly (to reduce jitter)
  if (abs(targetAngle - currentAngle) >= 2) {
    // Update the servo position
    switch (servoNumber) {
      case 1: servo1.write(targetAngle); break;
      case 2: servo2.write(targetAngle); break;
      case 3: servo3.write(targetAngle); break;
      case 4: servo4.write(targetAngle); break;
      case 5: servo5.write(targetAngle); break;
      case 6: servo6.write(targetAngle); break;
    }
    
    // Update the current angle
    currentAngles[servoNumber-1] = targetAngle;
    
    // Send feedback
    Serial.print("Servo ");
    Serial.print(servoNumber);
    Serial.print(" moved to ");
    Serial.print(targetAngle);
    Serial.println("°");
  }
}
