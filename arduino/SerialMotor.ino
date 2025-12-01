// This code must be uploaded to your ESP32 using the Arduino IDE.
// It simulates the motor controller logic that communicates with the Jetson.

#include <Arduino.h>

// Configuration must match the Python script
#define BAUD_RATE 115200

// Simple state variables to simulate the motor controller
String motor1_state = "STOPPED";
int motor1_speed = 0;
long motor1_position = 0; // Simulate NEMA-17 position in steps

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  Serial.println("ESP32 Motor Controller Simulator Initialized.");
  Serial.println("Ready to receive commands...");
}

void loop() {
  // Check if data is available to be read from the Jetson
  if (Serial.available() > 0) {
    // Read the incoming string until the newline character (\n)
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove leading/trailing whitespace

    if (command.length() > 0) {
      // Print the received command for debugging - USE ONLY IF NOT SENDING AN ACK IMMEDIATELY
      // Serial.print("Received command: "); 
      // Serial.println(command); 

      // --- COMMAND PROCESSING LOGIC ---

      if (command.startsWith("M1_START_SPEED_")) {
        // Example: M1_START_SPEED_200
        int speed_index = command.lastIndexOf('_') + 1;
        String speed_str = command.substring(speed_index);
        motor1_speed = speed_str.toInt();

        // Simulate activating the NEMA-17
        motor1_state = "RUNNING";

        // Send confirmation back to the Jetson
        Serial.print("M1_ACK_STARTED_@");
        Serial.println(motor1_speed); // e.g., M1_ACK_STARTED_@200

      } else if (command.equals("M1_STOP")) {
        // Simulate stopping the NEMA-17
        motor1_state = "STOPPED";
        motor1_speed = 0;
        
        // Send confirmation
        Serial.println("M1_ACK_STOPPED");

      } else if (command.equals("M1_GET_STATUS")) {
        // Simulate sending sensor/status data back to the Jetson
        motor1_position += motor1_speed * 10; // Simulate position change
        
        // Structure the data to send back
        String data = "M1_DATA|State:" + motor1_state + "|Speed:" + String(motor1_speed) + "|Position:" + String(motor1_position);

        // Send the data packet
        Serial.println(data);
        
      } else {
        // Unrecognized command
        Serial.println("ERROR: UNKNOWN_COMMAND");
      }
    }
  }

  // A small delay to prevent the loop from running too fast
  delay(10); 
}