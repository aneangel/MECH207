#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import time
import sys
import json

# RP2350 USB identifiers
RP2350_VID = 0x2886  # Seeed Technology Co., Ltd.
RP2350_PID = 0x0058  # XIAO RP2350
BAUD_RATE = 115200

def find_rp2350():
    """Find the RP2350 device automatically"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == RP2350_VID and port.pid == RP2350_PID:
            return port.device
    return None

def send_command(command_dict):
    """Send a JSON command to the microcontroller"""
    try:
        # Find the RP2350 device
        port = find_rp2350()
        if not port:
            print("Error: XIAO RP2350 not found!")
            print("Make sure the device is connected.")
            return
        
        print(f"Found RP2350 at: {port}")
        
        # Open serial connection
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
        time.sleep(2)  # Wait for serial connection to stabilize
        
        # Send command as plain text (matching Fan_PWM.cpp format)
        if command_dict["cmd"] == "pulse":
            command_str = "pulse"
        elif command_dict["cmd"] == "fan":
            if "state" in command_dict:
                command_str = "fan on" if command_dict["state"] else "fan off"
            elif "pwm" in command_dict:
                command_str = f"fan {command_dict['pwm']}"
            else:
                command_str = "fan off"
        else:
            command_str = ""
        
        ser.write(f'{command_str}\n'.encode())
        print(f"Command sent: {command_str}")
        
        # Read response from microcontroller
        time.sleep(0.1)
        while ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"MCU: {response}")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Make sure the device is connected and the port is correct")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 jetson_solenoid_control.py pulse")
        print("  python3 jetson_solenoid_control.py fan on")
        print("  python3 jetson_solenoid_control.py fan off")
        print("  python3 jetson_solenoid_control.py fan 128")
    else:
        # Parse command line arguments into JSON command
        if sys.argv[1] == "pulse":
            command = {"cmd": "pulse"}
        elif sys.argv[1] == "fan":
            if len(sys.argv) < 3:
                print("Error: 'fan' command requires a parameter (on/off/0-255)")
                sys.exit(1)
            
            if sys.argv[2] == "on":
                command = {"cmd": "fan", "state": True}
            elif sys.argv[2] == "off":
                command = {"cmd": "fan", "state": False}
            else:
                # Assume it's a PWM value
                try:
                    pwm_value = int(sys.argv[2])
                    command = {"cmd": "fan", "pwm": pwm_value}
                except ValueError:
                    print(f"Error: Invalid PWM value '{sys.argv[2]}'. Must be 0-255")
                    sys.exit(1)
        else:
            print(f"Error: Unknown command '{sys.argv[1]}'")
            sys.exit(1)
        
        send_command(command)
