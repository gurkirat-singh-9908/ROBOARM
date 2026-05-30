void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Wait for serial connection to be established
  while (!Serial) {
    ; // Wait for the serial port to connect
  }
}

void loop() {
  // Check if there is data available to read
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String receivedData = Serial.readStringUntil('\n');
    
    // Print the received data for debugging
    Serial.println("Received data: ");
    Serial.println(receivedData);
    
    // You can then split and process the received data if needed
  }
}
