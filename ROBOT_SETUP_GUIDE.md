# Autonomous Jetson Nano Robot with SLAM
## Complete Web-Based Mapping & Navigation System

Transform your Jetson Nano into an autonomous mapping robot with real-time web dashboard!

---

## 🎯 Features

- ✅ **Visual SLAM** - Real-time mapping with monocular camera
- ✅ **Web Dashboard** - Access from any device on network
- ✅ **Live Video Stream** - See what robot sees
- ✅ **Occupancy Grid Map** - 2D map of environment
- ✅ **Autonomous Navigation** - Point-and-click navigation
- ✅ **Motor Control** - Web-based and keyboard controls
- ✅ **Headless Operation** - Perfect for robot deployment
- ✅ **Real-time Metrics** - Position, speed, trajectory tracking
- ✅ **Map Saving** - Save and reload maps

---

## 📋 Requirements

### Hardware
1. **Jetson Nano** (2GB or 4GB)
2. **UGREEN 4K USB Camera** (or compatible)
3. **Motor Driver** (L298N, DRV8833, or similar)
4. **Robot Chassis** with 2 DC motors (differential drive)
5. **Power Supply** (5V 4A for Jetson + battery for motors)
6. **Jumper Wires** for connections

### Software
- JetPack 4.6+ (Ubuntu 18.04 or 20.04)
- Python 3.6+
- OpenCV 4.x
- Network connection (WiFi or Ethernet)

---

## 🚀 Installation

### Step 1: System Update
```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### Step 2: Install Dependencies
```bash
# Python and pip
sudo apt-get install -y python3-pip python3-dev

# OpenCV dependencies
sudo apt-get install -y libopencv-dev python3-opencv

# GPIO library for Jetson
sudo pip3 install Jetson.GPIO

# Web framework
sudo pip3 install flask flask-socketio

# Additional packages
sudo pip3 install numpy eventlet
```

### Step 3: Install Project Files
```bash
# Create project directory
mkdir ~/jetson_slam_robot
cd ~/jetson_slam_robot

# Copy all project files here
# - slam_web_server.py
# - motor_control.py
# - camera_calibration.py
# - templates/dashboard.html
```

### Step 4: Camera Calibration (Recommended)
```bash
# Print checkerboard pattern
wget https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png

# Run calibration
python3 camera_calibration.py

# Follow on-screen instructions
# This creates: camera_calibration.npz
```

### Step 5: Test Camera
```bash
# Check if camera is detected
ls /dev/video*

# Should show: /dev/video0 (or video1, etc.)

# Test camera access
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAILED')"
```

---

## 🔌 Hardware Setup

### Motor Driver Wiring (L298N Example)

**Jetson Nano → L298N Motor Driver:**
```
Pin 32 (PWM0)  → ENA (Left Motor PWM)
Pin 29         → IN1 (Left Motor Direction)
Pin 31         → IN2 (Left Motor Direction)

Pin 33 (PWM2)  → ENB (Right Motor PWM)
Pin 36         → IN3 (Right Motor Direction)
Pin 38         → IN4 (Right Motor Direction)

GND            → GND (Common ground)
```

**Motor Driver → Motors:**
```
OUT1, OUT2 → Left Motor
OUT3, OUT4 → Right Motor
```

**Power:**
```
12V Battery → 12V Input on L298N
5V Output   → (Optional) Can power sensors
```

### Pin Configuration

Default pin mapping (BOARD numbering):
```python
Left Motor:
  PWM: Pin 32
  IN1: Pin 29
  IN2: Pin 31

Right Motor:
  PWM: Pin 33
  IN1: Pin 36
  IN2: Pin 38
```

To change pins, edit `motor_control.py`:
```python
config = {
    'left_motor_pins': {
        'pwm': YOUR_PIN,
        'in1': YOUR_PIN,
        'in2': YOUR_PIN
    },
    # ... etc
}
```

---

## ⚙️ Configuration

### 1. Network Setup (Headless Access)

**Find Jetson IP:**
```bash
hostname -I
# Example output: 192.168.1.100
```

**Set Static IP (Optional):**
```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Add:
```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

Apply:
```bash
sudo netplan apply
```

### 2. Auto-Start on Boot

Create systemd service:
```bash
sudo nano /etc/systemd/system/slam-robot.service
```

Add:
```ini
[Unit]
Description=SLAM Robot Web Server
After=network.target

[Service]
Type=simple
User=your-username
WorkingDirectory=/home/your-username/jetson_slam_robot
ExecStart=/usr/bin/python3 slam_web_server.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable slam-robot.service
sudo systemctl start slam-robot.service
```

Check status:
```bash
sudo systemctl status slam-robot.service
```

---

## 🎮 Usage

### Starting the System

**Manual Start:**
```bash
cd ~/jetson_slam_robot
python3 slam_web_server.py
```

You should see:
```
======================================================================
SLAM System for Jetson Nano Robot
Web Dashboard for Headless Operation
======================================================================

Camera calibration loaded
SLAM system initialized
Camera initialized
Camera thread started

======================================================================
Web Dashboard Starting...
======================================================================

Access dashboard at:
  Local:   http://localhost:5000
  Network: http://192.168.1.100:5000

Press Ctrl+C to stop
======================================================================
```

### Accessing the Dashboard

**From Another Device:**
1. Connect to same network as Jetson
2. Open web browser
3. Navigate to: `http://<jetson-ip>:5000`
4. Example: `http://192.168.1.100:5000`

**From Jetson itself:**
- Open browser and go to `http://localhost:5000`

---

## 🕹️ Operating the Robot

### Web Dashboard Controls

**1. Control Modes:**
- **Manual** - Direct control via buttons/keyboard
- **Autonomous** - Navigate to clicked map positions
- **Mapping** - Autonomous exploration

**2. Movement Controls:**
- **Buttons** - Click direction buttons
- **Keyboard** - W/↑ (forward), S/↓ (back), A/← (left), D/→ (right), Space (stop)

**3. Map Features:**
- **White** - Free space (explored)
- **Black** - Obstacles (detected)
- **Gray** - Unknown (not explored)
- **Green Line** - Robot trajectory
- **Red Circle** - Current robot position
- **Blue Arrow** - Robot heading

**4. Functions:**
- **Reset SLAM** - Clear map and trajectory
- **Save Map** - Export current map and trajectory

### Keyboard Shortcuts
```
W or ↑    : Move Forward
S or ↓    : Move Backward
A or ←    : Turn Left
D or →    : Turn Right
Space     : Stop
```

---

## 🗺️ Autonomous Navigation

### Point-and-Click Navigation (Coming Soon)
1. Switch to "Autonomous" mode
2. Click target position on map
3. Robot navigates automatically
4. Avoids obstacles using SLAM data

### Exploration Mode
1. Switch to "Mapping" mode
2. Robot autonomously explores
3. Builds map of environment
4. Returns to start position

---

## 📊 Understanding the Dashboard

### Live Camera Feed
- Real-time view from robot camera
- Shows position and orientation overlay
- Used for visual odometry

### Real-Time Map
- 2D occupancy grid (top-down view)
- Updates as robot moves
- Shows obstacles and trajectory
- Grid: 500x500 pixels (25x25 meters at default scale)

### Robot Status Panel
- **Position X, Y** - Current location (meters)
- **Orientation** - Heading angle (degrees)
- **FPS** - Processing frame rate
- **Frames** - Total frames processed
- **Trajectory** - Number of trajectory points

### Trajectory Plot
- X-Y graph of robot path
- Green line shows movement
- Updates every 2 seconds

### Activity Log
- Real-time event logging
- Command history
- Status messages
- Timestamps for debugging

---

## 🔧 Troubleshooting

### Camera Issues

**Camera not detected:**
```bash
# Check USB connection
lsusb

# Check video devices
ls -l /dev/video*

# Try different camera index
# Edit slam_web_server.py, change:
camera = cv2.VideoCapture(0)  # Try 1, 2, etc.
```

**Low FPS:**
```bash
# Reduce resolution in slam_web_server.py:
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

### Motor Issues

**Motors not responding:**
```bash
# Test GPIO
sudo python3 -c "import Jetson.GPIO as GPIO; GPIO.setmode(GPIO.BOARD); print('GPIO OK')"

# Run motor test
python3 motor_control.py
```

**Wrong direction:**
- Swap IN1 and IN2 pins in code
- Or physically swap motor wires

### Web Dashboard Issues

**Cannot access dashboard:**
```bash
# Check if server is running
sudo netstat -tulpn | grep 5000

# Check firewall
sudo ufw status
sudo ufw allow 5000

# Ping Jetson from other device
ping 192.168.1.100
```

**Connection keeps dropping:**
- Check WiFi signal strength
- Use Ethernet for stability
- Reduce camera resolution

### SLAM Issues

**Tracking lost frequently:**
- Improve lighting
- Add texture to environment
- Calibrate camera properly
- Reduce movement speed

**Map not updating:**
- Check camera feed in dashboard
- Verify SLAM processing (check FPS)
- Look at activity log for errors

---

## 🎯 Performance Optimization

### For Better Performance:

**1. Reduce Camera Resolution:**
```python
# In slam_web_server.py
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

**2. Reduce Feature Count:**
```python
# In slam_web_server.py, VisualSLAM __init__
config = {
    'max_features': 1000,  # Default: 1500
}
```

**3. Enable Power Mode:**
```bash
# Max performance
sudo nvpmodel -m 0
sudo jetson_clocks
```

**4. Increase Swap:**
```bash
# Create 4GB swap
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

## 💾 Saving and Loading Maps

### Save Map
Click "Save Map" button in dashboard or:
```python
# Maps saved as:
map_<timestamp>.npy        # Occupancy grid
map_<timestamp>.jpg        # Visual map
trajectory_<timestamp>.json # Trajectory data
```

### Load Map (Manual)
```python
import numpy as np
import json

# Load occupancy grid
grid = np.load('map_1234567890.npy')

# Load trajectory
with open('trajectory_1234567890.json', 'r') as f:
    trajectory = json.load(f)
```

---

## 🔐 Security Notes

**Important:** The web dashboard has no authentication!

For production use:
1. Add authentication (Flask-Login)
2. Use HTTPS
3. Restrict network access
4. Use VPN for remote access

---

## 📚 Advanced Features

### Custom Motor Configuration

Edit `motor_control.py`:
```python
config = {
    'motor_driver': 'L298N',
    'left_motor_pins': {'pwm': 32, 'in1': 29, 'in2': 31},
    'right_motor_pins': {'pwm': 33, 'in1': 36, 'in2': 38},
    'pwm_frequency': 1000,
    'max_speed': 80,  # Limit max speed
    'min_speed': 25,  # Minimum to move
}
```

### Add Sensors (IMU, Ultrasonic, etc.)

Integrate in `slam_web_server.py`:
```python
# Example: Add IMU data
def read_imu():
    # Your IMU code
    return {'yaw': yaw, 'pitch': pitch, 'roll': roll}

# Use in SLAM pose estimation
```

### ROS Integration

```python
# Publish odometry to ROS
import rospy
from nav_msgs.msg import Odometry

def publish_odom(position):
    odom = Odometry()
    odom.pose.pose.position.x = position['x']
    odom.pose.pose.position.y = position['y']
    # ... etc
    pub.publish(odom)
```

---

## 📝 File Structure

```
jetson_slam_robot/
├── slam_web_server.py          # Main server
├── motor_control.py            # Motor control
├── camera_calibration.py       # Calibration tool
├── templates/
│   └── dashboard.html          # Web interface
├── camera_calibration.npz      # Camera calibration
├── map_*.npy                   # Saved maps
├── trajectory_*.json           # Saved trajectories
└── README.md                   # This file
```

---

## 🤝 Contributing

Improvements welcome! Areas to enhance:
- Better obstacle detection
- Loop closure detection
- Multi-robot SLAM
- Path planning algorithms
- Deep learning integration

---

## 📄 License

MIT License - Feel free to use and modify!

---

## 🆘 Support

Having issues? Check:
1. Troubleshooting section above
2. Activity log in dashboard
3. System logs: `journalctl -u slam-robot -f`
4. Test individual components first

---

## 🎓 Learn More

**Visual Odometry & SLAM:**
- [ORB-SLAM2 Paper](https://arxiv.org/abs/1610.06475)
- [Visual SLAM Tutorial](https://www.youtube.com/watch?v=2YrSz0d3rHw)

**Jetson Nano:**
- [NVIDIA Jetson Documentation](https://docs.nvidia.com/jetson/)
- [Jetson GPIO Library](https://github.com/NVIDIA/jetson-gpio)

**Robotics:**
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [Robot Navigation](https://automaticaddison.com/)

---

**Happy Mapping! 🗺️🤖**
