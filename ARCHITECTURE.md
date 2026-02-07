# System Architecture
## Jetson Nano Autonomous Robot with SLAM

## Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        JETSON NANO ROBOT                         │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Hardware Layer                         │  │
│  │                                                            │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐ │  │
│  │  │  Camera  │  │  Motors  │  │   GPIO   │  │  Power   │ │  │
│  │  │UGREEN 4K │  │ L298N x2 │  │   Pins   │  │  5V 4A   │ │  │
│  │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────────┘ │  │
│  └───────┼─────────────┼─────────────┼────────────────────────┘  │
│          │             │             │                            │
│  ┌───────▼─────────────▼─────────────▼────────────────────────┐  │
│  │              Software Layer - Python                        │  │
│  │                                                              │  │
│  │  ┌─────────────────────────────────────────────────────┐   │  │
│  │  │         Visual SLAM Engine                          │   │  │
│  │  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │   │  │
│  │  │  │ Feature  │  │  Pose    │  │  Occupancy Grid  │  │   │  │
│  │  │  │ Detection│→ │Estimation│→ │   Map Builder    │  │   │  │
│  │  │  └──────────┘  └──────────┘  └──────────────────┘  │   │  │
│  │  │                                                      │   │  │
│  │  │  ┌──────────────────────────────────────────────┐  │   │  │
│  │  │  │          Trajectory Storage                  │  │   │  │
│  │  │  │   (X, Y, Z, Yaw, Timestamp)                 │  │   │  │
│  │  │  └──────────────────────────────────────────────┘  │   │  │
│  │  └─────────────────────────────────────────────────────┘   │  │
│  │                          ▲                                  │  │
│  │                          │                                  │  │
│  │  ┌───────────────────────▼──────────────────────────────┐  │  │
│  │  │         Motor Control System                         │  │  │
│  │  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │  │  │
│  │  │  │  GPIO    │  │   PWM    │  │   Navigation     │   │  │  │
│  │  │  │ Control  │→ │ Generator│→ │   Controller     │   │  │  │
│  │  │  └──────────┘  └──────────┘  └──────────────────┘   │  │  │
│  │  └──────────────────────────────────────────────────────┘  │  │
│  │                          ▲                                  │  │
│  │                          │                                  │  │
│  │  ┌───────────────────────▼──────────────────────────────┐  │  │
│  │  │         Web Server (Flask + SocketIO)                │  │  │
│  │  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │  │  │
│  │  │  │   HTTP   │  │ WebSocket│  │  Video Stream    │   │  │  │
│  │  │  │  Routes  │  │  Events  │  │    (MJPEG)       │   │  │  │
│  │  │  └──────────┘  └──────────┘  └──────────────────┘   │  │  │
│  │  └──────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                                 │                                  │
│                         Network Interface                         │
│                         WiFi / Ethernet                           │
└─────────────────────────────────┼────────────────────────────────┘
                                  │
                          ┌───────▼────────┐
                          │   Network      │
                          │  192.168.x.x   │
                          └───────┬────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
            ┌───────▼────────┐         ┌───────▼────────┐
            │   Web Browser  │         │   Web Browser  │
            │   (Desktop)    │         │    (Mobile)    │
            │                │         │                │
            │  Dashboard UI  │         │  Dashboard UI  │
            └────────────────┘         └────────────────┘
```

## Data Flow

### 1. Camera Processing Pipeline
```
Camera → OpenCV → Grayscale → Feature Detection (ORB)
                              ↓
                         Feature Matching
                              ↓
                    Essential Matrix Estimation
                              ↓
                         Pose Recovery
                              ↓
                    Update Robot Position
                              ↓
                      Update Map & Trajectory
```

### 2. Web Communication Pipeline
```
Client (Browser) ─────────────┐
                              │
    ┌─────────────────────────▼──────────────────────────┐
    │              SocketIO Events                       │
    │                                                     │
    │  control      →  Motor commands                    │
    │  set_mode     →  Change operation mode             │
    │  reset_slam   →  Clear map & trajectory            │
    │  save_map     →  Export current map                │
    │                                                     │
    │  metrics      ←  Real-time status updates          │
    │  status       ←  System messages                   │
    └─────────────────────────────────────────────────────┘

HTTP Streaming:
    /video_feed   →  Live camera MJPEG stream
    /map_feed     →  Live map visualization MJPEG stream
    /api/status   →  JSON status data
    /api/trajectory → JSON trajectory data
```

### 3. Motor Control Flow
```
Web Command → SocketIO Event → RobotController
                                      ↓
                              MotorController
                                      ↓
                        ┌─────────────┴─────────────┐
                        ▼                           ▼
                  Left Motor PWM              Right Motor PWM
                        │                           │
                  ┌─────┴─────┐             ┌──────┴──────┐
                  │ IN1 │ IN2 │             │ IN3  │ IN4  │
                  └─────┴─────┘             └──────┴──────┘
                        │                           │
                   L298N Driver                L298N Driver
                        │                           │
                    DC Motor                    DC Motor
```

## Component Details

### Visual SLAM Component
```python
class VisualSLAM:
    - Camera calibration (intrinsic matrix)
    - ORB feature detector (1500 features)
    - Feature matcher (Brute Force)
    - Essential matrix estimation (RANSAC)
    - Pose recovery (R, t)
    - Occupancy grid (500x500 pixels)
    - Trajectory storage (list of poses)
```

**Features:**
- Real-time odometry estimation
- Occupancy grid mapping
- Trajectory tracking
- Thread-safe operation

**Update Rate:**
- Camera: 30 FPS
- SLAM Processing: 10-20 FPS (depends on features)
- Map Update: 10 FPS

### Motor Control Component
```python
class MotorController:
    - GPIO pin configuration
    - PWM generation (1000 Hz)
    - Direction control (H-bridge)
    - Speed control (0-100%)
    
    Methods:
    - move_forward(speed)
    - move_backward(speed)
    - turn_left(speed)
    - turn_right(speed)
    - rotate_left(speed)
    - rotate_right(speed)
    - stop()
```

**Control Modes:**
1. **Manual** - Direct user control
2. **Autonomous** - Navigate to target position
3. **Mapping** - Exploration mode

### Web Server Component
```python
Flask Application:
    Routes:
    - /              → Dashboard HTML
    - /video_feed    → Camera MJPEG stream
    - /map_feed      → Map MJPEG stream
    - /api/status    → JSON status
    - /api/trajectory → JSON trajectory
    
    SocketIO Events:
    - connect/disconnect
    - control (motor commands)
    - set_mode
    - reset_slam
    - save_map
    - metrics (emitted to clients)
```

**Threading:**
- Main thread: Flask/SocketIO server
- Camera thread: SLAM processing
- MJPEG generators: Video streaming

## System Requirements

### Computational Load
```
Component            CPU Usage    Memory    Notes
─────────────────────────────────────────────────────
Camera Capture       5-10%        ~100MB    30 FPS
Feature Detection    20-30%       ~200MB    ORB features
SLAM Processing      15-25%       ~150MB    Pose estimation
Web Server           5-10%        ~100MB    Flask + SocketIO
Video Streaming      10-15%       ~100MB    2 MJPEG streams
Motor Control        <5%          ~50MB     PWM + GPIO
─────────────────────────────────────────────────────
Total (Typical)      50-75%       ~700MB    On Jetson Nano 4GB
```

### Network Bandwidth
```
Stream               Bandwidth    Resolution   FPS
──────────────────────────────────────────────────
Camera Feed          2-4 Mbps     640x480      30
Map Feed             0.5-1 Mbps   400x400      10
WebSocket Data       <0.1 Mbps    JSON         Variable
──────────────────────────────────────────────────
Total                3-5 Mbps     
```

## File Organization

```
jetson_slam_robot/
│
├── slam_web_server.py          # Main application
│   ├── class VisualSLAM        # SLAM engine
│   ├── class RobotController   # High-level robot control
│   ├── Flask routes            # Web endpoints
│   └── SocketIO handlers       # Real-time events
│
├── motor_control.py            # Motor hardware interface
│   ├── class MotorController   # GPIO/PWM control
│   └── class AutonomousNav     # Navigation logic
│
├── templates/
│   └── dashboard.html          # Web interface
│       ├── HTML structure
│       ├── CSS styling
│       └── JavaScript (SocketIO client)
│
├── camera_calibration.py      # Calibration tool
├── requirements_robot.txt     # Python dependencies
├── install.sh                 # Installation script
└── ROBOT_SETUP_GUIDE.md       # Documentation
```

## Performance Optimization Tips

### For Better FPS
1. Reduce camera resolution (640x480)
2. Decrease max features (1000 instead of 1500)
3. Enable Jetson max performance mode
4. Use CUDA-accelerated OpenCV (if available)

### For Better Mapping
1. Calibrate camera properly
2. Improve environment lighting
3. Add texture to surfaces
4. Move robot slowly and steadily

### For Better Network
1. Use Ethernet instead of WiFi
2. Reduce JPEG quality for streams
3. Limit dashboard update rate
4. Use local network (not internet)

## Security Considerations

**Current Implementation:**
- ⚠️ No authentication
- ⚠️ No encryption (HTTP)
- ⚠️ Open to local network

**For Production:**
- ✅ Add Flask-Login authentication
- ✅ Use HTTPS with SSL certificates
- ✅ Implement access control
- ✅ Use VPN for remote access
- ✅ Rate limiting on API endpoints

## Extension Points

### Adding Sensors
```python
# In slam_web_server.py
class SensorManager:
    def read_ultrasonic(self):
        # Your sensor code
        pass
    
    def read_imu(self):
        # Your IMU code
        pass
    
    def update_slam_with_sensors(self):
        # Fuse sensor data with visual SLAM
        pass
```

### Adding Path Planning
```python
# Implement A* or RRT algorithm
class PathPlanner:
    def plan_path(self, start, goal, occupancy_grid):
        # Return list of waypoints
        pass
    
    def follow_path(self, waypoints):
        # Navigate through waypoints
        pass
```

### ROS Integration
```python
# Publish SLAM data to ROS topics
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid

def publish_to_ros():
    # Odometry publisher
    # Map publisher
    pass
```

## Debugging

### Enable Verbose Logging
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Monitor Performance
```python
# Add timing decorators
import time
from functools import wraps

def timing(f):
    @wraps(f)
    def wrap(*args, **kwargs):
        start = time.time()
        result = f(*args, **kwargs)
        end = time.time()
        print(f'{f.__name__} took {end-start:.3f}s')
        return result
    return wrap
```

### Check System Resources
```bash
# CPU usage
htop

# GPU usage (if applicable)
tegrastats

# Memory
free -h

# Network
iftop
```

## References

- OpenCV Documentation: https://docs.opencv.org/
- Flask Documentation: https://flask.palletsprojects.com/
- Flask-SocketIO: https://flask-socketio.readthedocs.io/
- Jetson GPIO: https://github.com/NVIDIA/jetson-gpio
- ORB-SLAM: https://github.com/raulmur/ORB_SLAM2
