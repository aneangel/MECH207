# Motor Controller Auto-Start Setup

This guide explains how to set up the motor controller to automatically start when the Jetson boots up.

## Overview

The motor controller daemon (`motor_controller_daemon.py`) runs in the background and:
- Automatically connects to the ESP32-S3 motor controller
- Monitors the RP2350 IR sensor board for STOP/RESET commands
- Enables motors and maintains the connection
- Restarts automatically if it crashes

## Installation

### Step 1: Make the installer executable
```bash
cd /home/user/directory/final
chmod +x install_service.sh
```

### Step 2: Run the installer
```bash
sudo bash install_service.sh
```

This will:
- Install the systemd service
- Enable it to start on boot
- Optionally start it immediately

## Service Management

### Start the service
```bash
sudo systemctl start air-hockey-motor
```

### Stop the service
```bash
sudo systemctl stop air-hockey-motor
```

### Restart the service
```bash
sudo systemctl restart air-hockey-motor
```

### Check service status
```bash
sudo systemctl status air-hockey-motor
```

### View live logs
```bash
sudo journalctl -u air-hockey-motor -f
```

### Disable auto-start (if needed)
```bash
sudo systemctl disable air-hockey-motor
```

### Re-enable auto-start
```bash
sudo systemctl enable air-hockey-motor
```

## Testing

After installation, reboot the Jetson to test:
```bash
sudo reboot
```

After reboot, check if the service is running:
```bash
sudo systemctl status air-hockey-motor
```

You should see:
- `Active: active (running)`
- Recent log entries showing successful connection

## Troubleshooting

### Service fails to start
Check the logs:
```bash
sudo journalctl -u air-hockey-motor -n 50
```

### USB devices not found
The service waits for USB devices to be ready. If it still fails:
1. Check USB connections: `lsusb`
2. Verify ESP32-S3 shows up: `ls /dev/ttyACM*`
3. Verify RP2350 shows up: `ls /dev/ttyACM*`
4. Restart the service: `sudo systemctl restart air-hockey-motor`

### Permission issues
Make sure the user has access to serial ports:
```bash
sudo usermod -a -G dialout user
```
Then log out and back in.

## Uninstalling

To remove the auto-start service:
```bash
sudo systemctl stop air-hockey-motor
sudo systemctl disable air-hockey-motor
sudo rm /etc/systemd/system/air-hockey-motor.service
sudo systemctl daemon-reload
```

## Manual Testing

You can still run the motor controller manually for testing/calibration:
```bash
# Stop the service first
sudo systemctl stop air-hockey-motor

# Run manually
python3 motor_controller.py

# When done, restart the service
sudo systemctl start air-hockey-motor
```

## What Happens on Boot

1. Jetson boots up
2. systemd starts the `air-hockey-motor` service
3. Service runs `motor_controller_daemon.py`
4. Daemon connects to ESP32-S3 and RP2350
5. Motors are enabled
6. IR sensor monitoring begins
7. System is ready - goals and coin insertions are automatically detected

The whole process takes about 5-10 seconds after boot.
