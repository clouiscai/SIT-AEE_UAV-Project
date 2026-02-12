import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SquareVelocityMission(Node):
    def __init__(self):
        super().__init__('square_velocity_mission')

        # ===============================
        # QoS (MAVROS sensor compatibility)
        # ===============================
        mavros_sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===============================
        # State
        # ===============================
        self.state = State()
        self.current_pose = None

        # ===============================
        # Subscribers
        # ===============================
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            mavros_sensor_qos
        )

        # ===============================
        # Publishers
        # ===============================
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # ===============================
        # Services
        # ===============================
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.land_srv = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.wait_for_services()

        # ===============================
        # Mission state
        # ===============================
        self.phase = 'STARTUP_DELAY'
        self.phase_start_ns = self.get_clock().now().nanoseconds
        self.takeoff_target = None

        # Control loop @ 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Square velocity mission node started")

    # ===============================
    # Callbacks
    # ===============================
    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    # ===============================
    # Helpers
    # ===============================
    def wait_for_services(self):
        self.arm_srv.wait_for_service()
        self.mode_srv.wait_for_service()
        self.land_srv.wait_for_service()

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_srv.call_async(req)

    def arm(self, value):
        req = CommandBool.Request()
        req.value = value
        self.arm_srv.call_async(req)

    def land(self):
        req = CommandTOL.Request()
        self.land_srv.call_async(req)

    def publish_velocity(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self.vel_pub.publish(msg)

    def publish_position(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pos_pub.publish(msg)

    def start_phase(self):
        self.phase_start_ns = self.get_clock().now().nanoseconds

    def phase_time(self):
        return (self.get_clock().now().nanoseconds - self.phase_start_ns) / 1e9

    # ===============================
    # Main control loop
    # ===============================
    def control_loop(self):
        # Guard conditions
        if not self.state.connected:
            return
        if self.current_pose is None:
            return

        # Local ENU position
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z

        # --------------------------------------------------
        if self.phase == 'STARTUP_DELAY':
            if self.phase_time() >= 2.0:
                self.phase = 'WAIT_FOR_GUIDED'
                self.get_logger().info("Startup delay complete. Requesting GUIDED...")

        elif self.phase == 'WAIT_FOR_GUIDED':
            if self.state.mode != 'GUIDED':
                if self.phase_time() >= 1.0:
                    self.get_logger().info("Requesting GUIDED mode...")
                    self.set_mode('GUIDED')
                    self.start_phase() # Reset timer for throttling
                return

            self.start_phase()
            self.phase = 'GUIDED_DELAY'
            self.get_logger().info("GUIDED confirmed. Waiting before arming...")

        elif self.phase == 'GUIDED_DELAY':
            if self.phase_time() >= 2.0:
                self.phase = 'WAIT_FOR_ARM'
                self.start_phase() # Initial throttle for arming

        elif self.phase == 'WAIT_FOR_ARM':
            if not self.state.armed:
                if self.phase_time() >= 1.0:
                    self.get_logger().info("Requesting Arm...")
                    self.arm(True)
                    self.start_phase() # Reset timer for throttling
                return

            self.start_phase()
            self.phase = 'TAKEOFF'
            self.get_logger().info("ARMED → TAKEOFF")

        elif self.phase == 'TAKEOFF':
            # Use velocity for takeoff to ensure simulator compatibility
            self.publish_velocity(0.0, 0.0, 0.5)
            if z >= 2.0:
                self.publish_velocity(0.0, 0.0, 0.0)
                self.start_phase()
                self.phase = 'FORWARD'
                self.get_logger().info("Target altitude reached → FORWARD")

        elif self.phase == 'FORWARD':
            self.publish_velocity(1.0, 0.0, 0.0)
            if self.phase_time() >= 5.0:
                self.start_phase()
                self.phase = 'RIGHT'
                self.get_logger().info("RIGHT")

        elif self.phase == 'RIGHT':
            self.publish_velocity(0.0, -1.0, 0.0)
            if self.phase_time() >= 5.0:
                self.start_phase()
                self.phase = 'BACK'
                self.get_logger().info("BACK")

        elif self.phase == 'BACK':
            self.publish_velocity(-1.0, 0.0, 0.0)
            if self.phase_time() >= 5.0:
                self.start_phase()
                self.phase = 'LEFT'
                self.get_logger().info("LEFT")

        elif self.phase == 'LEFT':
            self.publish_velocity(0.0, 1.0, 0.0)
            if self.phase_time() >= 5.0:
                self.publish_velocity(0.0, 0.0, 0.0)
                self.land()
                self.phase = 'LAND'
                self.get_logger().info("LAND")

        elif self.phase == 'LAND':
            if not self.state.armed:
                self.phase = 'DONE'

        elif self.phase == 'DONE':
            self.publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info("Mission complete")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = SquareVelocityMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()