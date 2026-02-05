import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

class DroneVisualizer(Node):
    def __init__(self):
        super().__init__('drone_visualizer')
        
        # Telemetry Subscriptions (QoS Optimized)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile_sensor_data)
        self.batt_sub = self.create_subscription(Float64, '/mavros/battery/voltage', self.batt_cb, qos_profile_sensor_data)
        self.alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.alt_cb, qos_profile_sensor_data)
        
        # Command Subscription (to move OFFBOARD CMD into visualization)
        self.cmd_sub = self.create_subscription(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self.cmd_cb, qos_profile_sensor_data)
        
        # State Data
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.mode = "UNKNOWN"
        self.armed = False
        self.battery = 0.0
        self.rel_alt = 0.0
        
        # Command Setpoints
        self.cmd_vx, self.cmd_vy, self.cmd_vz = 0.0, 0.0, 0.0
        
        self.history_x, self.history_y, self.history_z = [], [], []
        self.connected = False
        self.total_msgs = 0
        self.get_logger().info("Visualizer with High-Speed Telemetry Started.")

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.connected = True
        self.total_msgs += 1
        
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_z.append(self.z)
        if len(self.history_x) > 500:
            self.history_x.pop(0)
            self.history_y.pop(0)
            self.history_z.pop(0)

    def cmd_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_vz = msg.linear.z

    def state_cb(self, msg):
        self.mode = msg.mode
        self.armed = msg.armed

    def batt_cb(self, msg):
        self.battery = msg.data

    def alt_cb(self, msg):
        self.rel_alt = msg.data

def main():
    rclpy.init()
    node = DroneVisualizer()
    
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Text overlay for telemetry
    info_text = fig.text(0.02, 0.98, '', transform=fig.transFigure, verticalalignment='top', 
                         family='monospace', fontsize=10, bbox=dict(facecolor='white', alpha=0.8, edgecolor='blue'))

    try:
        while rclpy.ok():
            # High speed message processing
            rclpy.spin_once(node, timeout_sec=0.001)
            
            if node.connected:
                ax.cla()
                
                # Plot UAV
                ax.plot([node.x], [node.y], [node.z], 'ro', markersize=12, label='UAV', zorder=10)
                if len(node.history_x) > 1:
                    ax.plot(node.history_x, node.history_y, node.history_z, 'b-', alpha=0.4, linewidth=1.5, zorder=5)
                
                # Dynamic Limits
                side = 10
                ax.set_xlim(node.x - side, node.x + side)
                ax.set_ylim(node.y - side, node.y + side)
                ax.set_zlim(0, max(15, node.z + 5))
                
                # Build Robust Telemetry String
                arm_status = "ARMED" if node.armed else "DISARMED"
                telemetry_str = (
                    f"--- FLIGHT TELEMETRY ---\n"
                    f"MODE:    {node.mode}\n"
                    f"STATUS:  {arm_status}\n"
                    f"BATTERY: {node.battery:.2f} V\n"
                    f"ALTITUDE:{node.rel_alt:.2f} m\n"
                    f"POS:     X:{node.x:.1f} Y:{node.y:.1f} Z:{node.z:.1f}\n"
                    f"\n--- OFFBOARD CMD ---\n"
                    f"vx:      {node.cmd_vx:.2f} m/s\n"
                    f"vy:      {node.cmd_vy:.2f} m/s\n"
                    f"vz:      {node.cmd_vz:.2f} m/s\n"
                    f"\nLIVE MSGS: {node.total_msgs}"
                )
                info_text.set_text(telemetry_str)
                
                ax.set_xlabel('East (X)')
                ax.set_ylabel('North (Y)')
                ax.set_zlabel('Altitude (Z)')
                ax.grid(True, alpha=0.3)
                
                # High-speed refresh
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            else:
                ax.cla()
                ax.set_title("3D Visualizer - Waiting for MAVROS Link")
                ax.text(0.5, 0.5, 0.5, "INITIALIZING SYSTEM...\n(Check Terminal 1/2)", transform=ax.transAxes, ha='center')
                plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
