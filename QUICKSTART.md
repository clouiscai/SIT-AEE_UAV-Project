# UAV Flight Control - Quickstart

This project supports both a **real Pixhawk** (via serial) and a **simulated Pixhawk** for testing.

---

## 🚀 Option A: Real Pixhawk (Serial Connection)

You need **2–3 terminals**.

### 1. Start MAVROS (Communication Bridge)
Start this FIRST. Wait until you see heartbeat / connection messages.
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=serial:///dev/ttyAMA1:57600
```

### 2. Run the Autonomous Mission
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/cloui/Documents/GitHub/SIT-AEE_UAV-Project
python3 autonomous_square_mission.py
```
The script will automatically: set GUIDED mode → arm → takeoff → fly a square → land.

### 3. (Optional) Start 3D Visualizer
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/cloui/Documents/GitHub/SIT-AEE_UAV-Project
python3 visualizer.py
```

### ⚠️ Real Pixhawk Checklist
- Pixhawk must have a **valid position estimate** (GPS lock or other EKF source).
- Verify connection: `ros2 topic echo /mavros/state` → look for `connected: true`.
- Verify position: `ros2 topic echo /mavros/local_position/pose` → must have data.
- **Remove props** until you have verified the mission in a safe environment!

---

## 🚀 Option B: Simulated Pixhawk (Testing)

You need **4 terminals**.

### 1. Start MAVROS
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@ \
  -p gcs_url:=udp://@127.0.0.1:14555
```

### 2. Start Fake Pixhawk
```bash
cd /mnt/c/Users/cloui/Documents/GitHub/SIT-AEE_UAV-Project
python3 fake_pixhawk.py
```

### 3. Run the Mission or Manual Controller
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/cloui/Documents/GitHub/SIT-AEE_UAV-Project
python3 autonomous_square_mission.py
# OR for manual control:
# python3 manual_flight_controller.py
```

### 4. Start 3D Visualizer
```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/cloui/Documents/GitHub/SIT-AEE_UAV-Project
python3 visualizer.py
```

---

## 🎮 Manual Flight Controls (manual_flight_controller.py)

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

### 🛠️ Mode & Arming (Manual Mode Only)
- **G**: Set **GUIDED** Mode (Required for velocity control)
- **A**: **ARM** the motors (Required for movement)
- **D**: **DISARM**

## 🕹️ Manual Flight Procedure
1. Run all 4 terminals (Option B).
2. Press **G** (switch to Guided).
3. Press **A** (Arm motors).
4. Tap/Hold **Space** to take off.
5. Use **Arrow keys** to command lateral velocity.
6. Notice the **Blue Trail** in the Visualizer showing the resulting path.

## 🔍 Troubleshooting
- **Script hangs at startup**: MAVROS is not running or not connected. Check Terminal 1.
- **"Square velocity mission node started" but nothing else**: Pixhawk is not sending state or position data. Run `ros2 topic echo /mavros/state` and `ros2 topic echo /mavros/local_position/pose`.
- **Arming fails**: Pixhawk pre-arm checks are not passing (e.g. no GPS, no EKF).
