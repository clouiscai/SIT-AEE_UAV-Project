# UAV Manual Flight Simulator - Quickstart

This simulator allows you to control a virtual drone using **Velocity Commands (m/s)**. Holding a key sends a constant velocity setpoint to the drone, simulating how an autonomous system or joystick would interact with the Flight Controller.

## 🚀 Getting Started

To run the simulator, you need 4 separate terminal windows.

### 1. Start Fake Pixhawk
This terminal simulates the drone hardware and physics, reacting to velocity setpoints.
```bash
cd /mnt/c/Users/cloui/Downloads/"UAV Project"
python3 fake_pixhawk.py
```

### 2. Start MAVROS
The communication bridge.
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14550 -p gcs_url:=udp://@127.0.0.1:14555
```

### 3. Start Flight Controller
This translates your keyboard into **Velocity Setpoints**. Keep this window **focused**.
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/cloui/Downloads/"UAV Project"
python3 autonomous_square_mission.py
```

### 4. Start 3D Visualizer
Launches the Matplotlib 3D UI to see the drone point and trail.
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/cloui/Downloads/"UAV Project"
python3 visualizer.py
```

---

## 🎮 Velocity Controls (Terminal 3)

| Key | Action | Velocity Setpoint (Configurable) |
|-----|--------|-----------------------------------|
| **Arrow Up** | Move Forward | +5.0 m/s (Linear X) |
| **Arrow Down** | Move Backward | -5.0 m/s (Linear X) |
| **Arrow Left** | Move Left | +5.0 m/s (Linear Y) |
| **Arrow Right** | Move Right | -5.0 m/s (Linear Y) |
| **Space** | Climb (Up) | +2.5 m/s (Linear Z) |
| **Ctrl** | Descend (Down) | -2.5 m/s (Linear Z) |
| **Q / E** | Yaw Left / Right | 0.5 rad/s (Angular Z) |
| **S** | **INSTANT STOP** | 0.0 m/s (All Axes) |

### 🛠️ Mode & Arming
- **G**: Set **GUIDED** Mode (Required for velocity control)
- **A**: **ARM** the motors (Required for movement)
- **D**: **DISARM**

## 🕹️ Flight Procedure
1. Run all 4 terminals.
2. In **Terminal 3**, press **G** (switch to Guided).
3. Press **A** (Arm motors).
4. Tap/Hold **Space** to take off.
5. Use **Arrow keys** to command lateral velocity.
6. Notice the **Blue Trail** in the Visualizer showing the resulting path.

## 🔍 Raw Command Log
Terminal 3 logs the active MAVROS topic whenever velocity setpoints are non-zero:
`[/mavros/setpoint_velocity/cmd_vel_unstamped]`
