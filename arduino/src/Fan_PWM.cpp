#include <Arduino.h>

int FAN_pin = D6;    // Fan connected to digital pin D6

void setup() {
  Serial.begin(115200);
  pinMode(FAN_pin, OUTPUT);
  analogWrite(FAN_pin, 150);  // Start with fan off
  
  Serial.println("Fan PWM Controller Ready");
  Serial.println("Commands:");
  Serial.println("  fan <0-255> - set fan speed (e.g., 'fan 128')");
  Serial.println("  fan off - turn fan off");
  Serial.println("  fan on - turn fan full speed");
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
  }
}
