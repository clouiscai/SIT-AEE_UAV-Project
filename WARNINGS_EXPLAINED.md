# Understanding MAVROS Warnings

When you run the manual flight controller, you'll see various warnings in the terminal. Here's what they mean and whether you should worry about them:

## ✅ Safe to Ignore (Yellow Warnings)

### QoS Policy Warnings
```
[WARN] [mavros.global_position]: New subscription discovered on topic '/mavros/global_position/global', 
requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

**What it means:**
- ROS2 publishers and subscribers have Quality of Service (QoS) settings
- MAVROS uses different QoS settings than your node
- ROS2 is warning you they don't match perfectly

**Should you worry?** 
- ❌ **NO** - This is completely normal
- Messages still get through fine
- It's just ROS2 being overly cautious

**Why it happens:**
- MAVROS uses `BEST_EFFORT` reliability (faster, can drop messages)
- Your node uses `RELIABLE` (slower, guarantees delivery)
- They can still communicate, just not optimally

---

## ⚠️ Expected Errors (Red Errors)

### Timeout Errors
```
[ERROR] [mavros.rallypoint]: RP: timed out.
[ERROR] [mavros.geofence]: GF: timed out.
[WARN] [mavros.cmd]: CMD: Command 410 -- ack: timeout
```

**What it means:**
- MAVROS is trying to request advanced features from the Pixhawk
- Rally points (emergency landing spots)
- Geofences (virtual boundaries)
- Various commands the fake Pixhawk doesn't implement

**Should you worry?**
- ❌ **NO** - The fake Pixhawk is minimal
- It only implements basic features:
  - Heartbeat
  - IMU data
  - ARM/DISARM
  - Mode changes
  - Velocity commands

**Why it happens:**
- MAVROS automatically requests ALL features on startup
- The fake Pixhawk doesn't respond to these requests
- MAVROS times out and logs an error
- This doesn't affect basic flight simulation

---

## 🔧 How to Reduce Warnings (Optional)

If the warnings bother you, you can configure MAVROS to disable unused plugins:

### Create a MAVROS config file:
```yaml
# mavros_config.yaml
plugin_blacklist:
  - rallypoint
  - geofence
  - obstacle
  - trajectory
  - vision_pose
  - vision_speed
  - vibration
  - wheel_odometry
  - landing_target
```

### Launch MAVROS with config:
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@127.0.0.1:14555 \
  -p gcs_url:=udp://@127.0.0.1:14550 \
  -p system_id:=255 \
  -p component_id:=240 \
  --params-file mavros_config.yaml
```

**But honestly, it's easier to just ignore them!** 😊

---

## ✅ What You SHOULD See

### Terminal 1 (Fake Pixhawk) - Good Output:
```
Fake Pixhawk started
[FCU] Mode changed to GUIDED (custom_mode=4)
[FCU] Armed = True
```

### Terminal 3 (Manual Flight Controller) - Good Output:
```
[INFO] [manual_flight_controller]: Waiting for MAVROS services...
[INFO] [manual_flight_controller]: All services ready!
[INFO] [manual_flight_controller]: Manual Flight Controller initialized
[INFO] [manual_flight_controller]: Web UI available at http://localhost:8080
[INFO] [manual_flight_controller]: GUIDED mode command sent
[INFO] [manual_flight_controller]: ARM command sent
```

### Web UI - Good Status:
- Connection: **CONNECTED** (green)
- Mode: **GUIDED**
- Armed: **ARMED** (green)

---

## 🐛 Real Problems to Watch For

### ❌ Connection Issues:
```
[ERROR] [mavros]: FCU: Connection timeout
```
**Fix:** Restart Terminal 1 (fake Pixhawk)

### ❌ Service Not Available:
```
[ERROR] [manual_flight_controller]: Service /mavros/cmd/arming not available
```
**Fix:** Make sure Terminal 2 (MAVROS) is running

### ❌ File Not Found:
```
FileNotFoundError: [Errno 2] No such file or directory: 'ui/index.html'
```
**Fix:** Make sure you're running from the correct directory:
```bash
cd /mnt/c/Users/cloui/Downloads/"UAV Project"
```

---

## 📊 Summary

| Warning Type | Color | Ignore? | Reason |
|--------------|-------|---------|--------|
| QoS incompatible | Yellow | ✅ Yes | Normal ROS2 behavior |
| Rallypoint timeout | Red | ✅ Yes | Feature not implemented |
| Geofence timeout | Red | ✅ Yes | Feature not implemented |
| Command timeout | Yellow | ✅ Yes | Feature not implemented |
| Connection timeout | Red | ❌ No | Real problem! |
| Service unavailable | Red | ❌ No | MAVROS not running |

**Bottom line:** If the UI shows CONNECTED and you can control the drone, everything is working fine! 🎉
