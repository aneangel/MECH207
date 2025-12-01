import serial
import time
import sys

# --- CONFIGURATION ---
# IMPORTANT: Use the port confirmed in the previous step
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200 
TIMEOUT = 1 # Serial read timeout in seconds
# ---------------------

def send_command(ser, command):
    """Sends a command string to the serial port and reads the response."""
    
    # 1. Encode and send the command (add a newline character as a delimiter)
    try:
        command_to_send = command.encode('utf-8') + b'\n'
        print(f"Commander: Sending command: '{command}'")
        ser.write(command_to_send)
        
        # Give the ESP32 a moment to process and respond
        time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Error sending data: {e}")
        return None

    # 2. Read the response from the ESP32
    try:
        # ser.readline() waits until a newline character (\n) is received or timeout occurs
        response_bytes = ser.readline()
        
        if response_bytes:
            # Decode the bytes back into a readable string and strip whitespace
            response = response_bytes.decode('utf-8').strip()
            print(f"Controller: Received response: '{response}'")
            return response
        else:
            print("Controller: No response received within the timeout period.")
            return None
            
    except serial.SerialTimeoutException:
        print("Controller: Read timeout occurred. No response.")
        return None
    except serial.SerialException as e:
        print(f"Error reading data: {e}")
        return None


def main():
    """Main function to initialize serial and run the command simulation."""
    
    print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        # Initialize the serial connection
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT
        )
        print("Successfully connected to serial port.")
        
    except serial.SerialException as e:
        print(f"\nFATAL ERROR: Could not open serial port {SERIAL_PORT}.")
        print("Please ensure:")
        print("1. The ESP32 is plugged in.")
        print("2. The port name is correct.")
        print("3. You have permissions (did you log out and back in after adding to 'dialout' group?).")
        print(f"Error details: {e}")
        sys.exit(1)


    # --- SIMULATION LOOP ---
    
    # Command 1: Start Motor 1
    command_1 = "M1_START_SPEED_200"
    response_1 = send_command(ser, command_1)
    
    # Wait for a moment to simulate motor running
    print("\n--- Simulating 2 seconds of motor operation... ---\n")
    time.sleep(2)

    # Command 2: Request status/data from Motor 1 (simulating data feedback)
    command_2 = "M1_GET_STATUS"
    response_2 = send_command(ser, command_2)

    # Command 3: Stop Motor 1
    command_3 = "M1_STOP"
    response_3 = send_command(ser, command_3)

    # --- CLEANUP ---
    ser.close()
    print("\nSerial connection closed. Simulation complete.")

if __name__ == "__main__":
    # Ensure pyserial is installed: pip install pyserial
    main()