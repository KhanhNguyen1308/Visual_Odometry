#!/usr/bin/env python3
"""
SLAM System for Autonomous Jetson Nano Robot
Web-based Dashboard for Headless Operation

This system provides:
- Visual SLAM for mapping
- Real-time web dashboard
- Autonomous navigation capability
- Live camera feed
- 2D occupancy grid map

"""

import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import json
from collections import deque
import base64
from io import BytesIO
import os


class VisualSLAM:
    """Visual SLAM system with occupancy grid mapping"""
    
    def __init__(self, camera_matrix, dist_coeffs, config=None):
        """Initialize SLAM system"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        # Configuration
        default_config = {
            'max_features': 1500,
            'map_resolution': 0.05,  # meters per pixel
            'map_size': 500,  # pixels
            'min_matches': 10,
            'lowe_ratio': 0.75,
            'obstacle_threshold': 0.5,
        }
        self.config = {**default_config, **(config or {})}
        
        # Feature detector
        self.detector = cv2.ORB_create(
            nfeatures=self.config['max_features'],
            scaleFactor=1.2,
            nlevels=8
        )
        
        # Matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # State
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        # Pose
        self.R = np.eye(3, dtype=np.float64)
        self.t = np.zeros((3, 1), dtype=np.float64)
        self.yaw = 0.0  # Robot orientation
        
        # Trajectory
        self.trajectory = []
        self.trajectory.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0})
        
        # Occupancy grid map
        map_size = self.config['map_size']
        self.occupancy_grid = np.ones((map_size, map_size), dtype=np.float32) * 0.5
        self.map_center = map_size // 2
        
        # Landmarks (3D points)
        self.landmarks = []
        
        # Metrics
        self.frame_count = 0
        self.processing_times = deque(maxlen=30)
        self.is_lost = False
        
        # Thread safety
        self.lock = threading.Lock()
        
    def process_frame(self, frame):
        """Process frame and update SLAM state"""
        start_time = time.time()
        
        with self.lock:
            self.frame_count += 1
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect features
            keypoints, descriptors = self.detector.detectAndCompute(gray, None)
            
            if self.prev_frame is None:
                # Initialize
                self.prev_frame = gray
                self.prev_keypoints = keypoints
                self.prev_descriptors = descriptors
                metrics = self._get_metrics()
                return frame, metrics
            
            # Match features
            if descriptors is None or self.prev_descriptors is None:
                metrics = self._get_metrics()
                return frame, metrics
            
            matches = self.matcher.knnMatch(self.prev_descriptors, descriptors, k=2)
            
            # Ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < self.config['lowe_ratio'] * n.distance:
                        good_matches.append(m)
            
            if len(good_matches) >= self.config['min_matches']:
                # Extract points
                prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in good_matches])
                curr_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches])
                
                # Estimate motion
                R, t = self._estimate_motion(prev_pts, curr_pts)
                
                if R is not None and t is not None:
                    # Update pose
                    scale = 0.1  # Estimated scale (can be improved)
                    self.t = self.t + scale * self.R.dot(t)
                    self.R = R.dot(self.R)
                    
                    # Calculate yaw from rotation matrix
                    self.yaw = np.arctan2(self.R[1, 0], self.R[0, 0])
                    
                    # Add to trajectory
                    position = {
                        'x': float(self.t[0]),
                        'y': float(self.t[1]),
                        'z': float(self.t[2]),
                        'yaw': float(self.yaw)
                    }
                    self.trajectory.append(position)
                    
                    # Update map
                    self._update_occupancy_grid(prev_pts, curr_pts, good_matches)
                    
                    self.is_lost = False
                else:
                    self.is_lost = True
            else:
                self.is_lost = True
            
            # Update previous frame
            self.prev_frame = gray
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            
            # Metrics
            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)
            
            metrics = self._get_metrics()
            
        return frame, metrics
    
    def _estimate_motion(self, prev_pts, curr_pts):
        """Estimate camera motion from point correspondences"""
        E, mask = cv2.findEssentialMat(
            curr_pts, prev_pts,
            self.camera_matrix,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )
        
        if E is None:
            return None, None
        
        _, R, t, mask_pose = cv2.recoverPose(
            E, curr_pts, prev_pts, self.camera_matrix
        )
        
        return R, t
    
    def _update_occupancy_grid(self, prev_pts, curr_pts, matches):
        """Update occupancy grid based on feature depth estimation"""
        # Simple occupancy update based on robot position
        # Mark current position as free space
        pos = self.trajectory[-1]
        map_x = int(self.map_center + pos['x'] / self.config['map_resolution'])
        map_y = int(self.map_center + pos['y'] / self.config['map_resolution'])
        
        # Bounds check
        map_size = self.config['map_size']
        if 0 <= map_x < map_size and 0 <= map_y < map_size:
            # Mark as free (low occupancy)
            cv2.circle(self.occupancy_grid, (map_x, map_y), 3, 0.2, -1)
            
            # Ray tracing for obstacles (simplified)
            # This is a basic implementation - can be improved with depth estimation
            for i in range(0, len(prev_pts), 10):  # Sample every 10th point
                # Estimate rough distance based on disparity
                dx = curr_pts[i][0] - prev_pts[i][0]
                dy = curr_pts[i][1] - prev_pts[i][1]
                disparity = np.sqrt(dx**2 + dy**2)
                
                if disparity > 5:  # Potential obstacle
                    # Project in robot's direction
                    angle = pos['yaw']
                    distance = 1.0  # meters (estimated)
                    
                    obs_x = pos['x'] + distance * np.cos(angle)
                    obs_y = pos['y'] + distance * np.sin(angle)
                    
                    map_obs_x = int(self.map_center + obs_x / self.config['map_resolution'])
                    map_obs_y = int(self.map_center + obs_y / self.config['map_resolution'])
                    
                    if 0 <= map_obs_x < map_size and 0 <= map_obs_y < map_size:
                        self.occupancy_grid[map_obs_y, map_obs_x] = min(
                            self.occupancy_grid[map_obs_y, map_obs_x] + 0.1, 1.0
                        )
    
    def _get_metrics(self):
        """Get current metrics"""
        fps = 1.0 / np.mean(self.processing_times) if len(self.processing_times) > 0 else 0
        
        return {
            'frame_count': self.frame_count,
            'fps': fps,
            'trajectory_length': len(self.trajectory),
            'is_lost': self.is_lost,
            'position': self.trajectory[-1] if self.trajectory else {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}
        }
    
    def get_map_image(self):
        """Generate map visualization"""
        with self.lock:
            # Create RGB image from occupancy grid
            map_img = np.zeros((self.config['map_size'], self.config['map_size'], 3), dtype=np.uint8)
            
            # Color code: gray=unknown, white=free, black=occupied
            for i in range(self.config['map_size']):
                for j in range(self.config['map_size']):
                    occ = self.occupancy_grid[i, j]
                    if occ < 0.3:  # Free space
                        map_img[i, j] = [255, 255, 255]
                    elif occ > 0.7:  # Occupied
                        map_img[i, j] = [0, 0, 0]
                    else:  # Unknown
                        map_img[i, j] = [128, 128, 128]
            
            # Draw trajectory
            for i in range(1, len(self.trajectory)):
                prev = self.trajectory[i-1]
                curr = self.trajectory[i]
                
                prev_x = int(self.map_center + prev['x'] / self.config['map_resolution'])
                prev_y = int(self.map_center + prev['y'] / self.config['map_resolution'])
                curr_x = int(self.map_center + curr['x'] / self.config['map_resolution'])
                curr_y = int(self.map_center + curr['y'] / self.config['map_resolution'])
                
                cv2.line(map_img, (prev_x, prev_y), (curr_x, curr_y), (0, 255, 0), 2)
            
            # Draw robot position
            if self.trajectory:
                pos = self.trajectory[-1]
                robot_x = int(self.map_center + pos['x'] / self.config['map_resolution'])
                robot_y = int(self.map_center + pos['y'] / self.config['map_resolution'])
                
                # Robot body
                cv2.circle(map_img, (robot_x, robot_y), 8, (0, 0, 255), -1)
                
                # Direction indicator
                dir_len = 15
                end_x = int(robot_x + dir_len * np.cos(pos['yaw']))
                end_y = int(robot_y + dir_len * np.sin(pos['yaw']))
                cv2.arrowedLine(map_img, (robot_x, robot_y), (end_x, end_y), 
                               (255, 0, 0), 2, tipLength=0.3)
            
            # Grid lines
            for i in range(0, self.config['map_size'], 50):
                cv2.line(map_img, (i, 0), (i, self.config['map_size']), (100, 100, 100), 1)
                cv2.line(map_img, (0, i), (self.config['map_size'], i), (100, 100, 100), 1)
            
            return map_img
    
    def get_trajectory_data(self):
        """Get trajectory data for web display"""
        with self.lock:
            return list(self.trajectory)
    
    def reset(self):
        """Reset SLAM system"""
        with self.lock:
            self.R = np.eye(3, dtype=np.float64)
            self.t = np.zeros((3, 1), dtype=np.float64)
            self.yaw = 0.0
            self.trajectory = [{'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}]
            self.occupancy_grid[:] = 0.5
            self.is_lost = False


class RobotController:
    """Robot control interface (mock for demonstration)"""
    
    def __init__(self):
        self.mode = 'manual'  # manual, autonomous, mapping
        self.target_position = None
        self.is_moving = False
        
    def set_mode(self, mode):
        """Set robot operating mode"""
        self.mode = mode
        
    def move_forward(self, speed=0.3):
        """Move robot forward"""
        # Implement your motor control here
        print(f"Moving forward at speed {speed}")
        self.is_moving = True
        
    def move_backward(self, speed=0.3):
        """Move robot backward"""
        print(f"Moving backward at speed {speed}")
        self.is_moving = True
        
    def turn_left(self, speed=0.3):
        """Turn robot left"""
        print(f"Turning left at speed {speed}")
        self.is_moving = True
        
    def turn_right(self, speed=0.3):
        """Turn robot right"""
        print(f"Turning right at speed {speed}")
        self.is_moving = True
        
    def stop(self):
        """Stop robot"""
        print("Stopping robot")
        self.is_moving = False
        
    def set_target(self, x, y):
        """Set target position for autonomous navigation"""
        self.target_position = (x, y)
        print(f"Target set to ({x}, {y})")


# Global objects
app = Flask(__name__)
app.config['SECRET_KEY'] = 'jetson-nano-slam-2024'
socketio = SocketIO(app, cors_allowed_origins="*")

slam = None
robot = RobotController()
camera = None
camera_running = False


def load_calibration(filename="camera_calibration.npz"):
    """Load camera calibration"""
    if os.path.exists(filename):
        data = np.load(filename)
        return data['camera_matrix'], data['dist_coeffs']
    else:
        # Default calibration
        camera_matrix = np.array([[700, 0, 640], [0, 700, 360], [0, 0, 1]], dtype=np.float64)
        dist_coeffs = np.zeros(5, dtype=np.float64)
        return camera_matrix, dist_coeffs


def camera_thread():
    """Camera processing thread"""
    global camera_running, slam, camera
    
    while camera_running:
        if camera is None:
            time.sleep(0.1)
            continue
            
        ret, frame = camera.read()
        if not ret:
            time.sleep(0.1)
            continue
        
        # Process with SLAM
        processed_frame, metrics = slam.process_frame(frame)
        
        # Emit metrics to web clients
        socketio.emit('metrics', metrics)
        
        time.sleep(0.033)  # ~30 FPS


def generate_camera_frames():
    """Generate camera frames for streaming"""
    global camera, slam
    
    while True:
        if camera is None:
            time.sleep(0.1)
            continue
            
        ret, frame = camera.read()
        if not ret:
            continue
        
        # Resize for web
        frame = cv2.resize(frame, (640, 480))
        
        # Add overlay info
        pos = slam.trajectory[-1] if slam.trajectory else {'x': 0, 'y': 0, 'yaw': 0}
        cv2.putText(frame, f"Pos: ({pos['x']:.2f}, {pos['y']:.2f})", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Yaw: {np.degrees(pos['yaw']):.1f}°", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Mode: {robot.mode}", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Encode frame
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


def generate_map_frames():
    """Generate map frames for streaming"""
    global slam
    
    while True:
        if slam is None:
            time.sleep(0.1)
            continue
        
        map_img = slam.get_map_image()
        
        # Encode
        ret, buffer = cv2.imencode('.jpg', map_img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.1)  # 10 FPS for map


# Flask routes
@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_camera_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/map_feed')
def map_feed():
    """Map streaming route"""
    return Response(generate_map_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/status')
def get_status():
    """Get system status"""
    if slam is None:
        return jsonify({'error': 'SLAM not initialized'})
    
    pos = slam.trajectory[-1] if slam.trajectory else {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}
    
    return jsonify({
        'position': pos,
        'mode': robot.mode,
        'is_moving': robot.is_moving,
        'is_lost': slam.is_lost,
        'frame_count': slam.frame_count,
        'trajectory_length': len(slam.trajectory)
    })


@app.route('/api/trajectory')
def get_trajectory():
    """Get full trajectory"""
    if slam is None:
        return jsonify([])
    return jsonify(slam.get_trajectory_data())


# SocketIO event handlers
@socketio.on('connect')
def handle_connect():
    """Client connected"""
    print('Client connected')
    emit('status', {'message': 'Connected to robot'})


@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected"""
    print('Client disconnected')


@socketio.on('control')
def handle_control(data):
    """Handle robot control commands"""
    command = data.get('command')
    
    if command == 'forward':
        robot.move_forward()
    elif command == 'backward':
        robot.move_backward()
    elif command == 'left':
        robot.turn_left()
    elif command == 'right':
        robot.turn_right()
    elif command == 'stop':
        robot.stop()
    
    emit('status', {'message': f'Command {command} executed'})


@socketio.on('set_mode')
def handle_set_mode(data):
    """Set robot operating mode"""
    mode = data.get('mode')
    robot.set_mode(mode)
    emit('status', {'message': f'Mode set to {mode}'})


@socketio.on('reset_slam')
def handle_reset_slam():
    """Reset SLAM system"""
    if slam:
        slam.reset()
        emit('status', {'message': 'SLAM reset'})


@socketio.on('save_map')
def handle_save_map():
    """Save current map"""
    if slam:
        timestamp = int(time.time())
        
        # Save occupancy grid
        np.save(f'map_{timestamp}.npy', slam.occupancy_grid)
        
        # Save trajectory
        with open(f'trajectory_{timestamp}.json', 'w') as f:
            json.dump(slam.get_trajectory_data(), f, indent=2)
        
        # Save map image
        map_img = slam.get_map_image()
        cv2.imwrite(f'map_{timestamp}.jpg', map_img)
        
        emit('status', {'message': f'Map saved as map_{timestamp}'})


def main():
    global slam, camera, camera_running
    
    print("="*70)
    print("SLAM System for Jetson Nano Robot")
    print("Web Dashboard for Headless Operation")
    print("="*70)
    
    # Load calibration
    camera_matrix, dist_coeffs = load_calibration()
    print("\nCamera calibration loaded")
    
    # Initialize SLAM
    slam = VisualSLAM(camera_matrix, dist_coeffs)
    print("SLAM system initialized")
    
    # Initialize camera
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Cannot open camera")
        return
    
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("Camera initialized")
    
    # Start camera thread
    camera_running = True
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()
    print("Camera thread started")
    
    print("\n" + "="*70)
    print("Web Dashboard Starting...")
    print("="*70)
    print("\nAccess dashboard at:")
    print("  Local:   http://localhost:5000")
    print("  Network: http://<jetson-ip>:5000")
    print("\nPress Ctrl+C to stop")
    print("="*70 + "\n")
    
    # Start Flask app
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        camera_running = False
        if camera:
            camera.release()


if __name__ == '__main__':
    main()
