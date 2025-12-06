# Jetson Orin Nano Python Script
# This script reads sensor data and sends JSON commands to the XIAO RP2040.
# REQUIRES: pyserial (pip install pyserial)
# Commands are sent as structured JSON, e.g., '{"cmd":"fan", "pwm":180}\n'

import serial
import time
import threading
import sys
import json # New import for structured data

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200

# Global dictionary to store the latest data from the XIAO
latest_data = {
    'ir_sensor_1': 0,
    'ir_sensor_2': 0,
    'ir_sensor_3': 0,
    'solenoid_1_state': 0,
    'solenoid_2_state': 0
}

# --- Serial Communication Thread ---
def serial_reader_thread(ser):
    """Continuously reads data from the serial port and updates the global state."""
    print("Serial reader thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                # Read until a newline character
                line = ser.readline().decode('utf-8').strip()

                if not line:
                    continue

                # 1. Check for the simple CSV Data Feed (e.g., "1,0,1,0,1")
                if line.count(',') == 4 and all(c.isdigit() or c == ',' for c in line):
                    parts = line.split(',')
                    if len(parts) == 5:
                        try:
                            # Parse the data: IR1,IR2,IR3,SOL1,SOL2
                            global latest_data
                            latest_data['ir_sensor_1']      = int(parts[0])
                            latest_data['ir_sensor_2']      = int(parts[1])
                            latest_data['ir_sensor_3']      = int(parts[2])
                            latest_data['solenoid_1_state'] = int(parts[3])
                            latest_data['solenoid_2_state'] = int(parts[4])
                        except ValueError as e:
                            print(f"Error parsing CSV data line: {line} -> {e}")
                
                # 2. Check for JSON Responses (ACK/ERR) from the microcontroller
                elif line.startswith('{') and line.endswith('}'):
                    try:
                        response_obj = json.loads(line)
                        if response_obj.get('type') == 'response':
                            # Successful command response
                            print(f"MC Response [{response_obj['cmd']}]: {response_obj['status']} -> {response_obj.get('message', '')}")
                        elif response_obj.get('type') == 'status':
                            # Initialization or status report
                            print(f"MC Status: {response_obj.get('status')} -> {response_obj.get('message', '')}")
                        else:
                            print(f"MC: Unknown JSON: {line}")
                    except json.JSONDecodeError:
                        print(f"MC: Failed to decode JSON response: {line}")
                
                # 3. Catch any plain text output (should be minimal now)
                else:
                    print(f"MC: {line}")

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break
        
        time.sleep(0.005) 

# --- Command Functions ---

def send_command(ser, command_obj):
    """Converts a command object to a JSON string, adds a newline, and sends it."""
    # Convert Python dictionary to JSON string
    json_string = json.dumps(command_obj)
    full_command = json_string + '\n'
    
    try:
        ser.write(full_command.encode('utf-8'))
        print(f"Sent command: {json_string}")
    except serial.SerialException as e:
        print(f"Failed to send command: {e}")

def set_fan_pwm(ser, duty_cycle):
    """Sets the fan PWM duty cycle (0-255) using JSON."""
    if 0 <= duty_cycle <= 255:
        command_obj = {"cmd": "fan", "pwm": duty_cycle}
        send_command(ser, command_obj)
    else:
        print("Error: PWM duty cycle must be between 0 and 255.")

def actuate_solenoid(ser, solenoid_number):
    """Triggers the 2-second pulse for the specified solenoid (1 or 2) using JSON."""
    if solenoid_number in [1, 2]:
        command_obj = {"cmd": "solenoid", "solenoid": solenoid_number, "state": "H"}
        send_command(ser, command_obj)
    else:
        print("Error: Solenoid number must be 1 or 2.")

# --- Main Application Loop ---

def main():
    """Initializes serial communication and runs the main loop."""
    try:
        # 1. Initialize Serial Connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        
        # 2. Start the Serial Reading Thread
        reader_thread = threading.Thread(target=serial_reader_thread, args=(ser,), daemon=True)
        reader_thread.start()

        time.sleep(2) 
        
        # 3. Example Control and Monitoring Loop
        print("\n--- Starting Control and Monitoring Loop (JSON) ---")
        
        # Example 1: Trigger Solenoid 1 pulse
        actuate_solenoid(ser, 1)
        time.sleep(3) # Wait for the pulse to finish

        # Example 2: Cycle Fan Speed
        print("\nCycling fan speed (LOW)")
        set_fan_pwm(ser, 80)
        time.sleep(2)
        
        print("Cycling fan speed (HIGH)")
        set_fan_pwm(ser, 200)
        time.sleep(2)
        
        print("Cycling fan speed (OFF)")
        set_fan_pwm(ser, 0)
        time.sleep(1)

        # 4. Continuous Monitoring (replace this with your actual Jetson logic)
        print("\n--- Entering Continuous Monitoring Mode ---")
        print("Press Ctrl+C to exit.")
        
        while True:
            # Display the latest received data
            print(f"Feed: IR1={latest_data['ir_sensor_1']}, IR2={latest_data['ir_sensor_2']}, IR3={latest_data['ir_sensor_3']} | SOL1={latest_data['solenoid_1_state']}, SOL2={latest_data['solenoid_2_state']}", end='\r')
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"\n--- FATAL ERROR ---")
        print(f"Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()