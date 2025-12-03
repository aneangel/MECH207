#include <Arduino.h>

int FAN_pin = D6;       // Fan connected to digital pin D6
int SOLENOID_pin = D4;  // Solenoid connected to digital pin D4

void setup() {
  Serial.begin(115200);
  pinMode(FAN_pin, OUTPUT);
  pinMode(SOLENOID_pin, OUTPUT);
  analogWrite(FAN_pin, 150);  // Start with fan off
  digitalWrite(SOLENOID_pin, LOW);  // Solenoid off initially
  
  Serial.println("Fan & Solenoid Controller Ready");
  Serial.println("Commands:");
  Serial.println("  fan <0-255> - set fan speed (e.g., 'fan 128')");
  Serial.println("  fan off - turn fan off");
  Serial.println("  fan on - turn fan full speed");
  Serial.println("  pulse - trigger solenoid pulse (100ms)");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("fan ")) {
      String speedStr = command.substring(4);
      
      if (speedStr == "off") {
        analogWrite(FAN_pin, 0);
        Serial.println("Fan turned off");
      }
      else if (speedStr == "on") {
        analogWrite(FAN_pin, 255);
        Serial.println("Fan at full speed");
      }
      else {
        int speed = speedStr.toInt();
        if (speed >= 0 && speed <= 255) {
          analogWrite(FAN_pin, speed);
          Serial.print("Fan speed set to: ");
          Serial.println(speed);
        }
        else {
          Serial.println("Error: Fan speed must be 0-255");
        }
      }
    }
    else if (command == "pulse") {
      // Trigger solenoid pulse
      digitalWrite(SOLENOID_pin, HIGH);
      Serial.println("Solenoid pulse started");
      delay(100);  // 100ms pulse
      digitalWrite(SOLENOID_pin, LOW);
      Serial.println("Solenoid pulse complete");
    }
  }
}
