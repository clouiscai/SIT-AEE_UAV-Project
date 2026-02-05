import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import time

def get_ts():
    return time.strftime("[%H:%M:%S]")
from pynput import keyboard

class ManualFlightController(Node):
    def __init__(self):
        super().__init__('manual_flight_controller')
        
        # State variables
        self.state = State()
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.gps_position = NavSatFix()
        self.battery_voltage = 12.6
        self.altitude = 0.0
        
        # Manual control inputs (velocity commands in m/s)
        self.cmd_vel_x = 0.0  # Forward/Backward
        self.cmd_vel_y = 0.0  # Left/Right
        self.cmd_vel_z = 0.0  # Up/Down
        self.cmd_yaw_rate = 0.0  # Rotation
        
        # Control parameters
        self.max_velocity = 5.0  # m/s
        self.velocity_step = 0.5  # m/s increment per key press
        self.control_rate = 50  # Hz (typical drone control rate)
        
        # Keyboard state
        self.keys_pressed = set()
        self.actions_executed = set()  # Track which action keys have been executed
        
        # Subscribers with Sensor Data QoS (Best Effort) to match MAVROS
        self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/battery/voltage', self.battery_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_cb, qos_profile_sensor_data)
        
        # Publishers
        self.vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )
        
        # Service clients
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.arm_srv.wait_for_service()
        self.mode_srv.wait_for_service()
        self.get_logger().info('All services ready!')
        
        # High-frequency control loop (simulates autonomous controller)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        # Start keyboard listener
        self.start_keyboard_listener()
        
        self.get_logger().info('Manual Flight Controller initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Arrow Keys: Lateral movement (Forward/Back/Left/Right)')
        self.get_logger().info('  Space: Altitude UP')
        self.get_logger().info('  Ctrl: Altitude DOWN')
        self.get_logger().info('  Q/E: Yaw Left/Right')
        self.get_logger().info('  A: ARM')
        self.get_logger().info('  D: DISARM')
        self.get_logger().info('  G: Set GUIDED mode')
        self.get_logger().info('  S: STOP (zero all velocities)')
        
    def state_cb(self, msg):
        self.state = msg
        
    def pose_cb(self, msg):
        self.current_pose = msg
        
    def velocity_cb(self, msg):
        self.current_velocity = msg
        
    def gps_cb(self, msg):
        self.gps_position = msg
        
    def battery_cb(self, msg):
        self.battery_voltage = msg.data
        
    def altitude_cb(self, msg):
        self.altitude = msg.data
    
    def start_keyboard_listener(self):
        """Start listening to keyboard inputs"""
        def on_press(key):
            try:
                # Handle special keys
                if key == keyboard.Key.up:
                    self.keys_pressed.add('up')
                elif key == keyboard.Key.down:
                    self.keys_pressed.add('down')
                elif key == keyboard.Key.left:
                    self.keys_pressed.add('left')
                elif key == keyboard.Key.right:
                    self.keys_pressed.add('right')
                elif key == keyboard.Key.space:
                    self.keys_pressed.add('space')
                elif key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
                    self.keys_pressed.add('ctrl')
                # Handle character keys
                elif hasattr(key, 'char'):
                    if key.char:
                        self.keys_pressed.add(key.char.lower())
            except AttributeError:
                pass
        
        def on_release(key):
            try:
                # Handle special keys
                if key == keyboard.Key.up:
                    self.keys_pressed.discard('up')
                elif key == keyboard.Key.down:
                    self.keys_pressed.discard('down')
                elif key == keyboard.Key.left:
                    self.keys_pressed.discard('left')
                elif key == keyboard.Key.right:
                    self.keys_pressed.discard('right')
                elif key == keyboard.Key.space:
                    self.keys_pressed.discard('space')
                elif key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
                    self.keys_pressed.discard('ctrl')
                # Handle character keys
                elif hasattr(key, 'char'):
                    if key.char:
                        char_lower = key.char.lower()
                        self.keys_pressed.discard(char_lower)
                        # Clear action execution flag when key is released
                        self.actions_executed.discard(char_lower)
            except AttributeError:
                pass
        
        # Start listener in background thread
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.daemon = True
        listener.start()
    
    def update_velocity_commands(self):
        """Update velocity commands based on keyboard input"""
        # Reset velocities
        target_vel_x = 0.0
        target_vel_y = 0.0
        target_vel_z = 0.0
        target_yaw_rate = 0.0
        
        # Lateral movement (arrow keys)
        if 'up' in self.keys_pressed:
            target_vel_x = self.max_velocity  # Forward
        if 'down' in self.keys_pressed:
            target_vel_x = -self.max_velocity  # Backward
        if 'left' in self.keys_pressed:
            target_vel_y = self.max_velocity  # Left (Positive Y in FLU)
        if 'right' in self.keys_pressed:
            target_vel_y = -self.max_velocity # Right (Negative Y in FLU)
        
        # Altitude control
        if 'space' in self.keys_pressed:
            target_vel_z = self.max_velocity * 0.5  # Up (slower)
        if 'ctrl' in self.keys_pressed:
            target_vel_z = -self.max_velocity * 0.5  # Down (slower)
        
        # Yaw control
        if 'q' in self.keys_pressed:
            target_yaw_rate = 0.5  # Yaw left
        if 'e' in self.keys_pressed:
            target_yaw_rate = -0.5  # Yaw right
        
        # Smoothly interpolate current commands toward targets
        # This provides the "ramp up/slow down" effect
        accel = 0.08  # Lower = smoother, Higher = snappier
        
        self.cmd_vel_x = self.cmd_vel_x * (1 - accel) + target_vel_x * accel
        self.cmd_vel_y = self.cmd_vel_y * (1 - accel) + target_vel_y * accel
        self.cmd_vel_z = self.cmd_vel_z * (1 - accel) + target_vel_z * accel
        self.cmd_yaw_rate = self.cmd_yaw_rate * (1 - accel) + target_yaw_rate * accel
        
        # Absolute stop threshold
        if abs(self.cmd_vel_x) < 0.01: self.cmd_vel_x = 0.0
        if abs(self.cmd_vel_y) < 0.01: self.cmd_vel_y = 0.0
        if abs(self.cmd_vel_z) < 0.01: self.cmd_vel_z = 0.0
        if abs(self.cmd_yaw_rate) < 0.001: self.cmd_yaw_rate = 0.0
        
        # Handle action keys (one-time actions)
        # Only execute if key is pressed AND hasn't been executed yet
        if 'a' in self.keys_pressed and 'a' not in self.actions_executed:
            self.arm(True)
            self.actions_executed.add('a')
            self.get_logger().info('ARM command sent')
        
        if 'd' in self.keys_pressed and 'd' not in self.actions_executed:
            self.arm(False)
            self.actions_executed.add('d')
            self.get_logger().info('DISARM command sent')
        
        if 'g' in self.keys_pressed and 'g' not in self.actions_executed:
            self.set_mode('GUIDED')
            self.actions_executed.add('g')
            self.get_logger().info('GUIDED mode command sent')
        
        if 's' in self.keys_pressed and 's' not in self.actions_executed:
            self.cmd_vel_x = 0.0
            self.cmd_vel_y = 0.0
            self.cmd_vel_z = 0.0
            self.cmd_yaw_rate = 0.0
            self.actions_executed.add('s')
            print(f'{get_ts()} [FCU] EMERGENCY STOP SENT')
    
    def control_loop(self):
        """High-frequency control loop (50Hz) - simulates autonomous controller"""
        if not self.state.connected:
            return
        
        # Only update velocity commands from keyboard if keys are actually pressed
        # This allows browser API commands to work without being overwritten
        if len(self.keys_pressed) > 0:
            self.update_velocity_commands()
        
        # Create and publish velocity command (Twist unstamped)
        cmd = Twist()
        
        # Linear velocities (m/s)
        cmd.linear.x = self.cmd_vel_x
        cmd.linear.y = self.cmd_vel_y
        cmd.linear.z = self.cmd_vel_z
        
        # Angular velocities (rad/s)
        cmd.angular.z = self.cmd_yaw_rate
        
        # Publish at high frequency (like autonomous controller)
        self.vel_pub.publish(cmd)
        
        # Print topic name only when moving
        if abs(self.cmd_vel_x) > 0.01 or abs(self.cmd_vel_y) > 0.01 or abs(self.cmd_vel_z) > 0.01 or abs(self.cmd_yaw_rate) > 0.01:
            # We use the unstamped topic preferred for simple robot velocity control
            print(f'{get_ts()} [/mavros/setpoint_velocity/cmd_vel_unstamped]')
    
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_srv.call_async(req)
        print(f'{get_ts()} [/mavros/set_mode] custom_mode="{mode}"')
        return future
    
    def arm(self, arm_value=True):
        req = CommandBool.Request()
        req.value = arm_value
        future = self.arm_srv.call_async(req)
        print(f'{get_ts()} [/mavros/cmd/arming] value={arm_value}')
        return future
    



def main():
    rclpy.init()
    node = ManualFlightController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
