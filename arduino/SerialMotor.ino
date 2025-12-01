// Uploaded to ESP32 using the Arduino IDE.
// It simulates the motor controller logic that communicates with the Jetson.

#include <Arduino.h>

#define BAUD_RATE 115200

// Motor 1 pins (Vertical/Up-Down)
#define M1_STEP_PIN 16
#define M1_DIR_PIN 17
#define M1_ENABLE_PIN 18

// Motor 2 pins (Horizontal/Left-Right)
#define M2_STEP_PIN 19
#define M2_DIR_PIN 21
#define M2_ENABLE_PIN 22

// Motor 1 state
String motor1_state = "STOPPED";
volatile long motor1_position = 0;
int motor1_target_speed = 0;
unsigned long m1_last_step_micros = 0;

// Motor 2 state
String motor2_state = "STOPPED";
volatile long motor2_position = 0;
int motor2_target_speed = 0;
unsigned long m2_last_step_micros = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Configure Motor 1 pins
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENABLE_PIN, OUTPUT);
  digitalWrite(M1_ENABLE_PIN, LOW);
  digitalWrite(M1_DIR_PIN, HIGH);
  
  // Configure Motor 2 pins
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);
  digitalWrite(M2_ENABLE_PIN, LOW);
  digitalWrite(M2_DIR_PIN, HIGH);
  
  Serial.println("ESP32 Dual Motor Controller Initialized.");
  Serial.println("Ready to receive commands...");
}

void loop() {
  // Handle incoming commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      
      // ===== MOTOR 1 COMMANDS (Vertical) =====
      if (command.startsWith("M1_START_SPEED_")) {
        int speed_index = command.lastIndexOf('_') + 1;
        String speed_str = command.substring(speed_index);
        motor1_target_speed = abs(speed_str.toInt());
        
        digitalWrite(M1_DIR_PIN, speed_str.toInt() >= 0 ? HIGH : LOW);
        motor1_state = "RUNNING";
        Serial.print("M1_ACK_STARTED_@");
        Serial.println(motor1_target_speed);

      } else if (command.equals("M1_STOP")) {
        motor1_state = "STOPPED";
        motor1_target_speed = 0;
        Serial.println("M1_ACK_STOPPED");

      } else if (command.equals("M1_GET_STATUS")) {
        String data = "M1_DATA|State:" + motor1_state + 
                      "|Speed:" + String(motor1_target_speed) + 
                      "|Position:" + String(motor1_position);
        Serial.println(data);
      
      // ===== MOTOR 2 COMMANDS (Horizontal) =====
      } else if (command.startsWith("M2_START_SPEED_")) {
        int speed_index = command.lastIndexOf('_') + 1;
        String speed_str = command.substring(speed_index);
        motor2_target_speed = abs(speed_str.toInt());
        
        digitalWrite(M2_DIR_PIN, speed_str.toInt() >= 0 ? HIGH : LOW);
        motor2_state = "RUNNING";
        Serial.print("M2_ACK_STARTED_@");
        Serial.println(motor2_target_speed);

      } else if (command.equals("M2_STOP")) {
        motor2_state = "STOPPED";
        motor2_target_speed = 0;
        Serial.println("M2_ACK_STOPPED");

      } else if (command.equals("M2_GET_STATUS")) {
        String data = "M2_DATA|State:" + motor2_state + 
                      "|Speed:" + String(motor2_target_speed) + 
                      "|Position:" + String(motor2_position);
        Serial.println(data);
        
      } else {
        Serial.println("ERROR: UNKNOWN_COMMAND");
      }
    }
  }

  // Generate step pulses for Motor 1
  if (motor1_state == "RUNNING" && motor1_target_speed > 0) {
    unsigned long current_micros = micros();
    unsigned long step_interval = 1000000UL / motor1_target_speed;
    
    if (current_micros - m1_last_step_micros >= step_interval) {
      digitalWrite(M1_STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(M1_STEP_PIN, LOW);
      motor1_position++;
      m1_last_step_micros = current_micros;
    }
  }

  // Generate step pulses for Motor 2
  if (motor2_state == "RUNNING" && motor2_target_speed > 0) {
    unsigned long current_micros = micros();
    unsigned long step_interval = 1000000UL / motor2_target_speed;
    
    if (current_micros - m2_last_step_micros >= step_interval) {
      digitalWrite(M2_STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(M2_STEP_PIN, LOW);
      motor2_position++;
      m2_last_step_micros = current_micros;
    }
  }
}