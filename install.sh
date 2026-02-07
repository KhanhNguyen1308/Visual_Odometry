#!/bin/bash
# Installation script for Jetson Nano SLAM Robot
# Run with: bash install.sh

set -e

echo "======================================================================="
echo "Jetson Nano SLAM Robot - Installation Script"
echo "======================================================================="
echo ""

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "Warning: This does not appear to be a Jetson device."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "Step 1: Updating system..."
sudo apt-get update

# Install system dependencies
echo "Step 2: Installing system dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    libopencv-dev \
    build-essential \
    cmake \
    git

# Install Python packages
echo "Step 3: Installing Python packages..."
sudo pip3 install --upgrade pip
sudo pip3 install -r requirements_robot.txt

# Install Jetson.GPIO
echo "Step 4: Installing Jetson.GPIO..."
if ! python3 -c "import Jetson.GPIO" 2>/dev/null; then
    sudo pip3 install Jetson.GPIO
    
    # Add user to gpio group
    sudo groupadd -f -r gpio
    sudo usermod -a -G gpio $USER
    
    echo "Note: You may need to log out and back in for GPIO permissions to take effect"
fi

# Create templates directory
echo "Step 5: Setting up project structure..."
mkdir -p templates
mkdir -p maps
mkdir -p calibration_images

# Set permissions
echo "Step 6: Setting permissions..."
chmod +x slam_web_server.py
chmod +x motor_control.py
chmod +x camera_calibration.py

# Test imports
echo "Step 7: Testing installation..."
python3 << END
import cv2
import numpy as np
import flask
import flask_socketio
print("✓ OpenCV version:", cv2.__version__)
print("✓ NumPy version:", np.__version__)
print("✓ Flask installed")
print("✓ Flask-SocketIO installed")

try:
    import Jetson.GPIO as GPIO
    print("✓ Jetson.GPIO installed")
except ImportError:
    print("⚠ Jetson.GPIO not available (OK if testing on non-Jetson)")
END

echo ""
echo "======================================================================="
echo "Installation Complete!"
echo "======================================================================="
echo ""
echo "Next steps:"
echo "1. Camera Calibration (recommended):"
echo "   python3 camera_calibration.py"
echo ""
echo "2. Test Motors (optional):"
echo "   python3 motor_control.py"
echo ""
echo "3. Start SLAM System:"
echo "   python3 slam_web_server.py"
echo ""
echo "4. Access dashboard in browser:"
echo "   http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "For detailed instructions, see ROBOT_SETUP_GUIDE.md"
echo "======================================================================="
