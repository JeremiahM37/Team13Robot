#!/bin/bash
# Robot Control System - Installation Script
# Run with: bash install.sh

echo "=================================="
echo "Robot Control System - Installer"
echo "=================================="

cd "$(dirname "$0")"

# Update package list
echo ""
echo "Updating package list..."
sudo apt update

# Install system packages
echo ""
echo "Installing system packages..."
sudo apt install -y espeak python3-pip

# Install Python packages
echo ""
echo "Installing Python packages..."
pip install flask pyserial

# Download the real Pololu Maestro library
echo ""
echo "Downloading Pololu Maestro library..."
curl -L -o maestro.py https://raw.githubusercontent.com/FRC4564/Maestro/master/maestro.py

if [ -f maestro.py ]; then
    echo "Maestro library downloaded successfully."
else
    echo "WARNING: Could not download maestro.py"
    echo "You may need to download it manually from:"
    echo "  https://github.com/FRC4564/Maestro"
fi

echo ""
echo "=================================="
echo "Installation complete!"
echo "=================================="
echo ""
echo "Next steps:"
echo "  1. Run hardware test:  python hardware_test.py"
echo "  2. Start server:       python app.py"
echo "  3. Find your IP:       python show_ip.py"
echo ""
