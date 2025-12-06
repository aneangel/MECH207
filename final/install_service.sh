#!/bin/bash
# Install Air Hockey Motor Controller as a systemd service
# This will make it start automatically on boot

echo "======================================"
echo "Air Hockey Motor Controller Installer"
echo "======================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "This script needs root privileges to install the service."
    echo "Please run with sudo:"
    echo "  sudo bash install_service.sh"
    exit 1
fi

# Make daemon executable
echo "Making daemon executable..."
chmod +x /home/aneangel/MECH338/final/motor_controller_daemon.py

# Copy service file to systemd directory
echo "Installing service file..."
cp /home/aneangel/MECH338/final/air-hockey-motor.service /etc/systemd/system/

# Reload systemd to recognize new service
echo "Reloading systemd..."
systemctl daemon-reload

# Enable service to start on boot
echo "Enabling service to start on boot..."
systemctl enable air-hockey-motor.service

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""
echo "Service Commands:"
echo "  Start service:   sudo systemctl start air-hockey-motor"
echo "  Stop service:    sudo systemctl stop air-hockey-motor"
echo "  Restart service: sudo systemctl restart air-hockey-motor"
echo "  Check status:    sudo systemctl status air-hockey-motor"
echo "  View logs:       sudo journalctl -u air-hockey-motor -f"
echo "  Disable autostart: sudo systemctl disable air-hockey-motor"
echo ""
echo "The service will now start automatically on every boot."
echo ""
echo "Would you like to start the service now? (y/n)"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
    echo "Starting service..."
    systemctl start air-hockey-motor
    sleep 2
    echo ""
    systemctl status air-hockey-motor
fi
