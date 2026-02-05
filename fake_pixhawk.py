from pymavlink import mavutil
import time
import math

# ===============================
# MAVLink connection
# ===============================
master = mavutil.mavlink_connection(
    'udpout:127.0.0.1:14550',
    source_system=1,
    source_component=1
)

def ts():
    return time.strftime("[%H:%M:%S]")

print(f"{ts()} Fake Pixhawk FCU started")
print(f"{ts()} Waiting for MAVROS...")

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

        # 🔑 THIS IS THE CRITICAL LINE
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

        # HIGHRES_IMU (required by MAVROS)
        master.mav.highres_imu_send(
            boot_ms() * 1000,
            0.0, 0.0, 9.81,
            0.0, 0.0, cmd_yaw_rate,
            0.0, 0.0, 0.0,
            1013.25,
            0.0,
            0.0,
            25.0,
            0b111111111111
        )

        last_hb = now

    # -----------------------------------------------------
    # Position + attitude @ 50 Hz
    # -----------------------------------------------------
    if now - last_pub >= 0.02:
        # ENU → NED
        ned_x = pos_y
        ned_y = pos_x
        ned_z = -pos_z

        master.mav.local_position_ned_send(
            boot_ms(),
            ned_x, ned_y, ned_z,
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
    if not msg:
        continue

    mtype = msg.get_type()

    if mtype == "COMMAND_LONG":
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            armed = bool(msg.param1)
            print(f"{ts()} [FCU] ARMED = {armed}")
            master.mav.command_ack_send(
                msg.command,
                mavutil.mavlink.MAV_RESULT_ACCEPTED
            )

    elif mtype == "SET_MODE":
        if msg.custom_mode in MODE_NAMES:
            mode = MODE_NAMES[msg.custom_mode]
            print(f"{ts()} [FCU] MODE = {mode}")

    elif mtype == "SET_POSITION_TARGET_LOCAL_NED":
        # NED → ENU
        cmd_vx = msg.vy
        cmd_vy = msg.vx
        cmd_vz = -msg.vz
        cmd_yaw_rate = msg.yaw_rate
        # print(f"{ts()} [FCU] MOVE CMD: vx={cmd_vx:.2f} vy={cmd_vy:.2f} vz={cmd_vz:.2f}") # Too spammy if enabled always
        if boot_ms() % 1000 < 50: # Log roughly once per second
             print(f"{ts()} [FCU] Receiving Move Commands...")