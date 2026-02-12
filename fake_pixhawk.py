import os
# Force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil
import time
import math

# ===============================
# MAVLink connection
# ===============================
# We send TO MAVROS on 14550. MAVROS should be launched with fcu_url:=udp://@127.0.0.1:14550
master = mavutil.mavlink_connection(
    'udpout:127.0.0.1:14550',
    source_system=1,
    source_component=1
)

def ts():
    return time.strftime("[%H:%M:%S]")

print(f"{ts()} Fake Pixhawk FCU started (MAVLink 2.0)")
print(f"{ts()} Sending heartbeats to 127.0.0.1:14550...")
print(f"{ts()} Waiting for MAVROS commands...")

# =========================================================
# FCU state
# =========================================================
armed = False
mode = "STABILIZE"
custom_mode = 0

MODE_MAP = {
    "STABILIZE": 0,
    "GUIDED": 4,
}
MODE_NAMES = {v: k for k, v in MODE_MAP.items()}

# =========================================================
# Vehicle state (ENU internal)
# =========================================================
pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
vel_x, vel_y, vel_z = 0.0, 0.0, 0.0
yaw = 0.0

# Commanded velocities (ENU)
cmd_vx, cmd_vy, cmd_vz = 0.0, 0.0, 0.0
cmd_yaw_rate = 0.0

# =========================================================
# Timing
# =========================================================
start_time = time.time()
last_physics = start_time
last_hb = 0.0
last_pub = 0.0
home_sent = False

def boot_ms():
    return int((time.time() - start_time) * 1000) & 0xFFFFFFFF

# =========================================================
# Main loop
# =========================================================
while True:
    now = time.time()

    # -----------------------------------------------------
    # Physics update @ 50 Hz
    # -----------------------------------------------------
    if now - last_physics >= 0.02:
        dt = now - last_physics
        last_physics = now

        if armed and mode == "GUIDED":
            vel_x = 0.8 * vel_x + 0.2 * cmd_vx
            vel_y = 0.8 * vel_y + 0.2 * cmd_vy
            vel_z = 0.8 * vel_z + 0.2 * cmd_vz
            yaw += cmd_yaw_rate * dt
        else:
            vel_x *= 0.5
            vel_y *= 0.5
            vel_z *= 0.5

        pos_x += vel_x * dt
        pos_y += vel_y * dt
        pos_z = max(0.0, pos_z + vel_z * dt)

    # -----------------------------------------------------
    # HEARTBEAT + SYS_STATUS @ 1 Hz
    # -----------------------------------------------------
    if now - last_hb >= 1.0:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if mode == "GUIDED":
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

        custom_mode = MODE_MAP[mode]

        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            custom_mode,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )

        master.mav.sys_status_send(
            0x0001, 0x0001, 0x0001,
            500, 12000, 10,
            100, 0, 0, 0, 0, 0, 0
        )

        if not home_sent:
            master.mav.home_position_send(
                473589000, 85480000, 0,
                0, 0, 0,
                [0, 0, 0, 1],
                0, 0, 0
            )
            home_sent = True

        master.mav.highres_imu_send(
            boot_ms() * 1000,
            0.0, 0.0, 9.81,
            0.0, 0.0, cmd_yaw_rate,
            0.0, 0.0, 0.0,
            1013.25, 0.0, 0.0, 25.0,
            0b111111111111
        )
        if boot_ms() < 5000: # Log heartbeats for the first 5 seconds
             print(f"{ts()} [TX] Sending Heartbeat...")
        last_hb = now

    # -----------------------------------------------------
    # Position + attitude @ 50 Hz
    # -----------------------------------------------------
    if now - last_pub >= 0.02:
        master.mav.local_position_ned_send(
            boot_ms(),
            pos_y, pos_x, -pos_z,
            vel_y, vel_x, -vel_z
        )
        master.mav.attitude_send(
            boot_ms(),
            0.0, 0.0, yaw,
            0.0, 0.0, cmd_yaw_rate
        )
        last_pub = now

    # -----------------------------------------------------
    # MAVLink RX
    # -----------------------------------------------------
    msg = master.recv_match(blocking=False)
    if msg:
        mtype = msg.get_type()
        
        # Log incoming requests for debugging
        if mtype not in ["HEARTBEAT", "SET_POSITION_TARGET_LOCAL_NED"]:
             print(f"{ts()} [RX] {mtype}")

        if mtype == "COMMAND_LONG":
            # Always ACK any command to keep MAVROS happy
            master.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)

            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                armed = bool(msg.param1)
                print(f"{ts()} [FCU] ARMED = {armed}")
            
            elif msg.command == mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                print(f"{ts()} [FCU] Sending Autopilot Version...")
                master.mav.autopilot_version_send(
                    mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
                    1, 0, 0, 0,
                    bytearray([0]*8), bytearray([0]*8), bytearray([0]*8),
                    0, 0, 0, bytearray([0]*18)
                )

        elif mtype == "PARAM_REQUEST_LIST":
            print(f"{ts()} [FCU] Param list requested (sending empty)")
            # Send completion of param list (0 total)
            master.mav.param_value_send(b"DUMMY_PARAM", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 0, 0)
        
        elif mtype == "MISSION_REQUEST_LIST":
            print(f"{ts()} [FCU] Mission list requested (sending 0)")
            master.mav.mission_count_send(msg.get_srcSystem(), msg.get_srcComponent(), 0)

        elif mtype == "SET_MODE":
            if msg.custom_mode in MODE_NAMES:
                mode = MODE_NAMES[msg.custom_mode]
                print(f"{ts()} [FCU] MODE = {mode}")

        elif mtype == "SET_POSITION_TARGET_LOCAL_NED":
            cmd_vx = msg.vy
            cmd_vy = msg.vx
            cmd_vz = -msg.vz
            cmd_yaw_rate = msg.yaw_rate
            if boot_ms() % 1000 < 50:
                 print(f"{ts()} [FCU] Receiving Move Commands...")
    
    time.sleep(0.001) # Small sleep to prevent 100% CPU usage
