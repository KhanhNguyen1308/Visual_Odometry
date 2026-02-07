#!/usr/bin/env python3
"""
Motor Control Integration for Jetson Nano Robot
Supports common motor driver boards:
- L298N Motor Driver
- DRV8833 Motor Driver
- Adafruit Motor HAT
- GPIO-based motor control
"""
import time
try:
    import Jetson.GPIO as GPIO
    JETSON_GPIO_AVAILABLE = True
except ImportError:
    print("Warning: Jetson.GPIO not available. Using simulation mode.")
    JETSON_GPIO_AVAILABLE = False


class MotorController:
    """Motor controller for differential drive robot"""
    
    def __init__(self, config=None):
        """
        Initialize motor controller
        
        Args:
            config: Motor configuration dictionary
        """
        # Default configuration for L298N motor driver
        default_config = {
            'motor_driver': 'L298N',  # L298N, DRV8833, or GPIO
            'left_motor_pins': {
                'pwm': 32,      # Pin 32 (PWM0)
                'in1': 29,      # Direction pin 1
                'in2': 31       # Direction pin 2
            },
            'right_motor_pins': {
                'pwm': 33,      # Pin 33 (PWM2)
                'in1': 36,      # Direction pin 1
                'in2': 38       # Direction pin 2
            },
            'pwm_frequency': 1000,  # Hz
            'max_speed': 100,        # Percentage (0-100)
            'min_speed': 30,         # Minimum speed to overcome friction
        }
        
        self.config = {**default_config, **(config or {})}
        
        self.left_pwm = None
        self.right_pwm = None
        self.is_initialized = False
        
        if JETSON_GPIO_AVAILABLE:
            self._setup_gpio()
    
    def _setup_gpio(self):
        """Setup GPIO pins for motor control"""
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            # Setup left motor pins
            left_pins = self.config['left_motor_pins']
            GPIO.setup(left_pins['pwm'], GPIO.OUT)
            GPIO.setup(left_pins['in1'], GPIO.OUT)
            GPIO.setup(left_pins['in2'], GPIO.OUT)
            
            # Setup right motor pins
            right_pins = self.config['right_motor_pins']
            GPIO.setup(right_pins['pwm'], GPIO.OUT)
            GPIO.setup(right_pins['in1'], GPIO.OUT)
            GPIO.setup(right_pins['in2'], GPIO.OUT)
            
            # Create PWM objects
            self.left_pwm = GPIO.PWM(left_pins['pwm'], self.config['pwm_frequency'])
            self.right_pwm = GPIO.PWM(right_pins['pwm'], self.config['pwm_frequency'])
            
            # Start PWM with 0 duty cycle
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            self.is_initialized = True
            print("Motor controller initialized successfully")
            
        except Exception as e:
            print(f"Error initializing GPIO: {e}")
            self.is_initialized = False
    
    def _set_left_motor(self, speed, direction):
        """
        Set left motor speed and direction
        
        Args:
            speed: Speed percentage (0-100)
            direction: 1 for forward, -1 for backward, 0 for stop
        """
        if not self.is_initialized:
            print(f"[SIMULATION] Left motor: speed={speed}, direction={direction}")
            return
        
        pins = self.config['left_motor_pins']
        
        # Clamp speed
        speed = max(0, min(100, abs(speed)))
        
        if direction > 0:  # Forward
            GPIO.output(pins['in1'], GPIO.HIGH)
            GPIO.output(pins['in2'], GPIO.LOW)
        elif direction < 0:  # Backward
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.HIGH)
        else:  # Stop
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.LOW)
        
        # Set PWM duty cycle
        if self.left_pwm:
            self.left_pwm.ChangeDutyCycle(speed)
    
    def _set_right_motor(self, speed, direction):
        """
        Set right motor speed and direction
        
        Args:
            speed: Speed percentage (0-100)
            direction: 1 for forward, -1 for backward, 0 for stop
        """
        if not self.is_initialized:
            print(f"[SIMULATION] Right motor: speed={speed}, direction={direction}")
            return
        
        pins = self.config['right_motor_pins']
        
        # Clamp speed
        speed = max(0, min(100, abs(speed)))
        
        if direction > 0:  # Forward
            GPIO.output(pins['in1'], GPIO.HIGH)
            GPIO.output(pins['in2'], GPIO.LOW)
        elif direction < 0:  # Backward
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.HIGH)
        else:  # Stop
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.LOW)
        
        # Set PWM duty cycle
        if self.right_pwm:
            self.right_pwm.ChangeDutyCycle(speed)
    
    def move_forward(self, speed=50):
        """Move robot forward"""
        speed = max(self.config['min_speed'], min(self.config['max_speed'], speed))
        self._set_left_motor(speed, 1)
        self._set_right_motor(speed, 1)
    
    def move_backward(self, speed=50):
        """Move robot backward"""
        speed = max(self.config['min_speed'], min(self.config['max_speed'], speed))
        self._set_left_motor(speed, -1)
        self._set_right_motor(speed, -1)
    
    def turn_left(self, speed=50):
        """Turn robot left (differential drive)"""
        speed = max(self.config['min_speed'], min(self.config['max_speed'], speed))
        self._set_left_motor(speed * 0.3, -1)  # Left motor slow backward
        self._set_right_motor(speed, 1)         # Right motor forward
    
    def turn_right(self, speed=50):
        """Turn robot right (differential drive)"""
        speed = max(self.config['min_speed'], min(self.config['max_speed'], speed))
        self._set_left_motor(speed, 1)          # Left motor forward
        self._set_right_motor(speed * 0.3, -1)  # Right motor slow backward
    
    def rotate_left(self, speed=40):
        """Rotate robot in place (left)"""
        speed = max(self.config['min_speed'], min(self.config['max_speed'], speed))
        self._set_left_motor(speed, -1)
        self._set_right_motor(speed, 1)
    
    def rotate_right(self, speed=40):
        """Rotate robot in place (right)"""
        speed = max(self.config['min_speed'], min(self.config['max_speed'], speed))
        self._set_left_motor(speed, 1)
        self._set_right_motor(speed, -1)
    
    def stop(self):
        """Stop all motors"""
        self._set_left_motor(0, 0)
        self._set_right_motor(0, 0)
    
    def set_speed(self, left_speed, right_speed):
        """
        Set individual motor speeds
        
        Args:
            left_speed: Left motor speed (-100 to 100)
            right_speed: Right motor speed (-100 to 100)
        """
        # Left motor
        left_dir = 1 if left_speed > 0 else (-1 if left_speed < 0 else 0)
        self._set_left_motor(abs(left_speed), left_dir)
        
        # Right motor
        right_dir = 1 if right_speed > 0 else (-1 if right_speed < 0 else 0)
        self._set_right_motor(abs(right_speed), right_dir)
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        if self.is_initialized and JETSON_GPIO_AVAILABLE:
            self.stop()
            if self.left_pwm:
                self.left_pwm.stop()
            if self.right_pwm:
                self.right_pwm.stop()
            GPIO.cleanup()
            print("Motor controller cleaned up")


class AutonomousNavigator:
    """Autonomous navigation using SLAM data"""
    
    def __init__(self, motor_controller, slam):
        """
        Initialize autonomous navigator
        
        Args:
            motor_controller: MotorController instance
            slam: VisualSLAM instance
        """
        self.motor = motor_controller
        self.slam = slam
        self.is_navigating = False
        self.target_position = None
        
    def navigate_to(self, target_x, target_y):
        """
        Navigate to target position
        
        Args:
            target_x: Target X coordinate (meters)
            target_y: Target Y coordinate (meters)
        """
        self.target_position = (target_x, target_y)
        self.is_navigating = True
        
        print(f"Navigating to ({target_x}, {target_y})")
        
        # Simple navigation loop
        while self.is_navigating:
            # Get current position
            current_pos = self.slam.trajectory[-1] if self.slam.trajectory else None
            if not current_pos:
                time.sleep(0.1)
                continue
            
            # Calculate distance and angle to target
            dx = target_x - current_pos['x']
            dy = target_y - current_pos['y']
            distance = np.sqrt(dx**2 + dy**2)
            target_angle = np.arctan2(dy, dx)
            
            # Calculate heading error
            heading_error = target_angle - current_pos['yaw']
            
            # Normalize angle to [-pi, pi]
            while heading_error > np.pi:
                heading_error -= 2 * np.pi
            while heading_error < -np.pi:
                heading_error += 2 * np.pi
            
            # Check if reached target
            if distance < 0.1:  # 10cm threshold
                print("Target reached!")
                self.motor.stop()
                self.is_navigating = False
                break
            
            # Control logic
            if abs(heading_error) > 0.3:  # ~17 degrees
                # Rotate to face target
                if heading_error > 0:
                    self.motor.rotate_left(35)
                else:
                    self.motor.rotate_right(35)
            else:
                # Move forward
                # Adjust speed based on distance
                speed = min(60, 30 + distance * 20)
                self.motor.move_forward(speed)
            
            time.sleep(0.1)
    
    def stop_navigation(self):
        """Stop autonomous navigation"""
        self.is_navigating = False
        self.motor.stop()
        print("Navigation stopped")
    
    def explore_mode(self, duration=60):
        """
        Exploration mode - random movement for mapping
        
        Args:
            duration: Exploration duration in seconds
        """
        print(f"Starting exploration mode for {duration} seconds")
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            if not self.is_navigating:
                break
            
            # Check for obstacles in occupancy grid
            # If obstacle ahead, turn
            # Otherwise, move forward
            
            # Simple behavior: move forward for random time, then turn
            forward_time = np.random.uniform(2, 5)
            turn_time = np.random.uniform(1, 2)
            
            # Move forward
            self.motor.move_forward(40)
            time.sleep(forward_time)
            
            # Turn random direction
            if np.random.random() > 0.5:
                self.motor.rotate_left(35)
            else:
                self.motor.rotate_right(35)
            time.sleep(turn_time)
        
        self.motor.stop()
        print("Exploration complete")


# Test function
def test_motors():
    """Test motor controller"""
    print("Testing motor controller...")
    
    # Initialize
    motor = MotorController()
    
    try:
        # Test forward
        print("Forward...")
        motor.move_forward(50)
        time.sleep(2)
        
        # Test backward
        print("Backward...")
        motor.move_backward(50)
        time.sleep(2)
        
        # Test left turn
        print("Left turn...")
        motor.turn_left(50)
        time.sleep(2)
        
        # Test right turn
        print("Right turn...")
        motor.turn_right(50)
        time.sleep(2)
        
        # Test stop
        print("Stop...")
        motor.stop()
        time.sleep(1)
        
        print("Test complete!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        motor.cleanup()


if __name__ == "__main__":
    test_motors()
