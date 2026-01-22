#include <Servo.h>

Servo s1; 
Servo s2; 
Servo s3; 
Servo s4; 
Servo s5; 
Servo s6; 

// Variables to store the received values
int slider1 = 0, slider2 = 0, slider3 = 0, slider4 = 0, slider5 = 0, slider6 = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud
  s1.attach(2); 
  s2.attach(3);
  s3.attach(4);
  s4.attach(5);
  s5.attach(6);
  s6.attach(7);
  s1.write(90);
  s2.write(90);
  s3.write(90);
  s4.write(90);
  s5.write(90);
  s6.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim(); // Remove any trailing spaces

    // Parse and assign values
    sscanf(receivedData.c_str(), "%d %d %d %d %d %d", 
           &slider1, &slider2, &slider3, &slider4, &slider5, &slider6);

    // Debugging: Print the parsed values
    Serial.print("Slider values: ");
    Serial.print(slider1); Serial.print(" ");
    Serial.print(slider2); Serial.print(" ");
    Serial.print(slider3); Serial.print(" ");
    Serial.print(slider4); Serial.print(" ");
    Serial.print(slider5); Serial.print(" ");
    Serial.println(slider6);
    s1.write(slider2); 
    s2.write(slider3);
    s3.write(slider4);
    s4.write(slider5);
    s5.write(slider6);
    s6.write(slider1);
  }
}
