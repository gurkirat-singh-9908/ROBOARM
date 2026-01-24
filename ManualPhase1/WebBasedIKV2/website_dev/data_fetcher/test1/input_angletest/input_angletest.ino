void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Read until newline
    data.trim();  // Remove extra whitespace

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

    // Check checksum and send response
    if (checksum == value1 + value2) {
      Serial.println(String(value1) + " " + String(value2) + " T");
    } else {
      Serial.println(String(value1) + " " + String(value2) + " F");
    }
  }
}
