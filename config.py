# ─── Named Poses (degrees) ─────────────────────────────────
HOME = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0}

# 물체 운반용 공통 전환 자세 — 실제 로봇 조립 후 캘리브레이션 필요
CARRY = {"J1": 90.0, "J2": 100.0, "J3": 120.0, "J4": 90.0, "J5": 90.0}

# ─── CAN Bus (CANable Pro ↔ MCP2515) ───────────────────────
CAN_CHANNEL = "can0"
CAN_BUSTYPE = "socketcan"
CAN_BITRATE = 500000

# CAN Message IDs — RPi5 → Arduino Uno
CAN_ID_SERVO_MOVE = 0x10   # [servo_id:u8, position:u16]
CAN_ID_SERVO_READ = 0x11   # [servo_id:u8]
CAN_ID_GRIPPER    = 0x12   # [angle:u8]
CAN_ID_TOF_READ   = 0x13   # []
CAN_ID_ESTOP      = 0xFF   # []

# CAN Message IDs — Arduino Uno → RPi5
CAN_ID_SERVO_POS  = 0x20   # [servo_id:u8, position:u16]
CAN_ID_TOF_DIST   = 0x21   # [distance:u16]

# ─── STS3215 Servo ──────────────────────────────────────────
SERVO_CENTER = 2048
SERVO_STEPS_PER_DEG = 4096.0 / 360.0  # ~11.378

JOINTS = {
    "J1": {"servo_id": 1, "min": 0,  "max": 180, "home": 90.0,  "dir": 1},
    "J2": {"servo_id": 2, "min": 75, "max": 150, "home": 120.0, "dir": 1},
    "J3": {"servo_id": 3, "min": 0,  "max": 170, "home": 150.0, "dir": 1},
    "J4": {"servo_id": 4, "min": 30, "max": 150, "home": 80.0,  "dir": 1},
    "J5": {"servo_id": 5, "min": 0,  "max": 180, "home": 90.0,  "dir": 1},
}

# ─── Gripper (SG90) ─────────────────────────────────────────
GRIPPER_OPEN = 90
GRIPPER_CLOSE = 30

# ─── IK ─────────────────────────────────────────────────────
IK_L2 = 245.0  # upper arm (mm)
IK_L3 = 212.0  # forearm (mm)

# ─── Work Surface ───────────────────────────────────────────
WORK_SURFACE_Z = -150.0  # mm below base, 조립 후 실측
PRE_GRASP_Z = WORK_SURFACE_Z + 40.0
TOF_GRASP_DIST = 30  # mm — 이 거리 이하면 그리퍼 닫기
DESCEND_STEP_MM = 5.0

# ─── Bins (J1 degrees) ──────────────────────────────────────
BIN_BOLT_J1 = 45.0
BIN_NUT_J1 = 135.0

# ─── Camera (ArduCam, eye-in-hand) ──────────────────────────
CAMERA_ID = 0
CALIB_FILE = "camera_calib.npz"
HOMOGRAPHY_FILE = "homography.npz"

# ─── YOLO ────────────────────────────────────────────────────
YOLO_CONF = 0.7
CLASS_NAMES = {0: "bolt", 1: "nut"}

# ─── Trajectories ───────────────────────────────────────────
TRAJECTORY_DIR = "trajectories"
SCAN_TRAJ = "scan_path.csv"
BOLT_PLACE_TRAJ = "bolt_place.csv"
NUT_PLACE_TRAJ = "nut_place.csv"

# ─── S-curve Motion ─────────────────────────────────────────
MOVE_DURATION = 1.0      # seconds, default
MOVE_DURATION_FAST = 0.3  # short moves (descend steps)
CONTROL_PERIOD_MS = 20    # ms per control step
