#!/usr/bin/env python3
"""
Autonomous Square Mission - NO GPS REQUIRED
============================================
Based on the x500mavros pattern: blocking service calls with result checking,
RC safety overrides, and timer-based phase control.

Sequence: GUIDED → ARM → TAKEOFF → FLY SQUARE → LAND

Usage:
  1. Start MAVROS:  ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyAMA1:57600
  2. Run this:       python3 autonomous_square_no_gps.py
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist
import sys


class SquareNoGPS(Node):
    def __init__(self):
        super().__init__('square_no_gps')

        # ===============================
        # Configuration
        # ===============================
        self.TAKEOFF_ALT = 1.0       # meters (for takeoff command)
        self.HOVER_TIME = 5.0        # seconds to hover after takeoff
        self.SIDE_TIME = 5.0         # seconds per square side
        self.SIDE_SPEED = 1.0        # m/s lateral speed

        # ===============================
        # State tracking
        # ===============================
        self.connected = False
        self.armed = False
        self.mode = ""

        self.myModeSetting = ""
        self.myArmSetting = False
        self.alarmed = False

        # ===============================
        # Subscriber
        # ===============================
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # ===============================
        # Publisher (velocity commands for square flight)
        # ===============================
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # ===============================
        # Service clients
        # ===============================
        self.setmode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.setmode_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for set mode service...")

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for arming service...")

        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for takeoff service...")

        self.get_logger().info("All MAVROS services ready.")

    # ===============================
    # State callback with RC safety
    # ===============================
    def state_callback(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode
        self.connected = msg.connected

        # SAFETY: RC disarm detected → abort
        if (self.myArmSetting) and \
           (not self.armed) and \
           (self.myModeSetting.upper() != "LAND"):
            self.get_logger().warn("RC disarm detected, program terminated!")
            rclpy.shutdown()
            sys.exit(0)

        # SAFETY: RC land detected → abort
        if (self.myModeSetting != "") and \
           (self.myModeSetting.upper() != "LAND") and \
           (self.mode == "LAND") and \
           (self.armed):
            self.get_logger().warn("RC landing request detected, program terminated!")
            rclpy.shutdown()
            sys.exit(0)

    # ===============================
    # Blocking service calls (x500mavros style)
    # ===============================
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        async_call = self.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(self, async_call)

        if async_call.result().mode_sent:
            self.get_logger().info(f"Mode changed to {mode}")
            self.myModeSetting = mode
            return True
        else:
            self.get_logger().error("Failed to change mode!")
            return False

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        async_call = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, async_call)

        if async_call.result().success:
            self.get_logger().info("Drone armed successfully!")
            self.myArmSetting = True
            return True
        else:
            self.get_logger().error("Failed to arm the drone!")
            return False

    def takeoff(self):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = self.TAKEOFF_ALT
        async_call = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, async_call)

        if async_call.result().success:
            self.get_logger().info(f"Takeoff command sent OK! (alt={self.TAKEOFF_ALT}m)")
            return True
        else:
            self.get_logger().error(f"Failed to takeoff! {async_call.result()}")
            return False

    # ===============================
    # Timer helper (x500mavros style)
    # ===============================
    def start_timer(self, x):
        self.alarmed = False
        self.create_timer(x, self.timer_callback)

    def timer_callback(self):
        self.alarmed = True

    # ===============================
    # Velocity publisher
    # ===============================
    def publish_velocity(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def fly_phase(self, label, vx, vy, vz, duration):
        """Fly at given velocity for duration seconds, then stop."""
        self.get_logger().info(f"--- {label} ({duration}s) ---")
        self.alarmed = False
        self.start_timer(duration)
        while not self.alarmed:
            self.publish_velocity(vx, vy, vz)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Stop
        self.publish_velocity(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    drone = SquareNoGPS()

    # ===================================
    # 1. Wait for MAVROS connection
    # ===================================
    while not drone.connected:
        drone.get_logger().info("Waiting for MAVLink connection...")
        rclpy.spin_once(drone, timeout_sec=1)

    drone.get_logger().info("MAVLink connected!")

    # ===================================
    # 2. Set GUIDED mode
    # ===================================
    if not drone.set_mode("GUIDED"):
        rclpy.shutdown()
        exit()

    # ===================================
    # 3. Arm
    # ===================================
    if not drone.armed:
        if not drone.arm():
            rclpy.shutdown()
            exit()

    # ===================================
    # 4. Takeoff
    # ===================================
    if not drone.takeoff():
        rclpy.shutdown()
        exit()

    # Wait for takeoff to complete
    drone.get_logger().info(f"Hovering for {drone.HOVER_TIME}s after takeoff...")
    drone.start_timer(drone.HOVER_TIME)
    while not drone.alarmed:
        drone.get_logger().info("Lifting off...")
        rclpy.spin_once(drone, timeout_sec=1)

    # ===================================
    # 5. Fly a square (velocity-based)
    # ===================================
    drone.get_logger().info("=== STARTING SQUARE PATTERN ===")

    # Forward (+X)
    drone.fly_phase("FORWARD (+X)", drone.SIDE_SPEED, 0.0, 0.0, drone.SIDE_TIME)

    # Right (-Y)
    drone.fly_phase("RIGHT (-Y)", 0.0, -drone.SIDE_SPEED, 0.0, drone.SIDE_TIME)

    # Back (-X)
    drone.fly_phase("BACK (-X)", -drone.SIDE_SPEED, 0.0, 0.0, drone.SIDE_TIME)

    # Left (+Y)
    drone.fly_phase("LEFT (+Y)", 0.0, drone.SIDE_SPEED, 0.0, drone.SIDE_TIME)

    drone.get_logger().info("=== SQUARE COMPLETE ===")

    # ===================================
    # 6. Land
    # ===================================
    if not drone.set_mode("LAND"):
        rclpy.shutdown()
        exit()

    drone.get_logger().info("Landing... waiting for disarm.")
    rclpy.spin(drone)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
