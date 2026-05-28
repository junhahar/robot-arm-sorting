# ─── Named Poses (degrees) ─────────────────────────────────
HOME = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0}
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
CAN_ID_FSR_READ   = 0x14   # []
CAN_ID_ESTOP      = 0xFF   # []

# CAN Message IDs — Arduino Uno → RPi5
CAN_ID_SERVO_POS  = 0x20   # [servo_id:u8, position:u16]
CAN_ID_TOF_DIST   = 0x21   # [distance:u16]
CAN_ID_FSR_VAL    = 0x22   # [value:u16]

# ─── STS3215 Servo ──────────────────────────────────────────
SERVO_CENTER = 2048
SERVO_STEPS_PER_DEG = 4096.0 / 360.0  # ~11.378

JOINTS = {
    "J1": {"servo_id": 1, "min": 0,   "max": 270, "home": 90.0,  "dir": 1},
    "J2": {"servo_id": 2, "min": 0,   "max": 270, "home": 120.0, "dir": 1},
    "J3": {"servo_id": 3, "min": 0,   "max": 270, "home": 150.0, "dir": 1},
    "J4": {"servo_id": 4, "min": -90, "max": 270, "home": 80.0,  "dir": 1},
    "J5": {"servo_id": 5, "min": 0,   "max": 270, "home": 90.0,  "dir": 1},
}

# ─── Gripper (SG90) ─────────────────────────────────────────
GRIPPER_OPEN = 90
GRIPPER_CLOSE = 30
GRIP_CLOSE_WAIT = 0.5  # seconds

# ─── FSR (압력 센서) ────────────────────────────────────────
FSR_THRESHOLD = 100  # 아날로그 값, 조립 후 실측

# ─── IK ─────────────────────────────────────────────────────
IK_L2 = 245.0  # upper arm (mm)
IK_L3 = 212.0  # forearm (mm)

# IK joint angle mapping (internal rad <-> servo deg)
IK_J2_OFFSET = 75.0
IK_J2_RANGE_DEG = 150.0
IK_J3_OFFSET = 90.0
IK_J3_RANGE_DEG = 135.0
IK_J4_OFFSET = 90.0
IK_J4_RANGE_DEG = 120.0
GRIP_OFFSET_RAD = -1.5708  # -pi/2, 수직 파지. 조립 후 실측

# ─── Work Surface ───────────────────────────────────────────
WORK_SURFACE_Z = -150.0  # mm below base, 조립 후 실측
PRE_GRASP_Z = WORK_SURFACE_Z + 40.0
GRASP_DIST = 30       # mm — ToF 이 거리 이하면 그리퍼 닫기
PLACE_DIST = 15       # mm — bin 내부 놓기 거리
DESCEND_STEP_MM = 5.0
LIFT_HEIGHT = 40.0    # mm — 파지 후 들어올리기
MAX_PLACE_DEPTH = 120.0  # mm — bin 최대 하강 깊이
MAX_DESCEND_DEPTH = 200.0  # mm — 작업대 최대 하강

# ─── ArUco ──────────────────────────────────────────────────
ARUCO_DICT = "DICT_4X4_50"
ENV_ARUCO_ID = 0       # 환경 보정용 마커 ID
BIN_ARUCO = {
    "bolt":   {"id": 1, "offset_x": 0.0, "offset_y": 50.0},
    "nut":    {"id": 2, "offset_x": 0.0, "offset_y": 50.0},
    "washer": {"id": 3, "offset_x": 0.0, "offset_y": 50.0},
}

# ─── Camera (ArduCam OV9782, eye-in-hand) ───────────────────
CAMERA_ID = 0
CALIB_FILE = "camera_calib.npz"
# 핀홀 카메라 파라미터 — 체커보드 캘리브레이션 후 실측값으로 교체
CAM_FX = 600.0
CAM_FY = 600.0
CAM_CX = 320.0
CAM_CY = 240.0
# 카메라 마운트 오프셋 (그리퍼 기준, mm) — 오른쪽+안쪽 틸트
CAM_OFFSET_X = 30.0   # 그리퍼 기준 오른쪽 (mm), 조립 후 실측
CAM_OFFSET_Y = 0.0    # 그리퍼 기준 전방 (mm), 조립 후 실측

# ─── Visual Servoing ────────────────────────────────────────
GRIPPER_PX = 350      # 그리퍼 중심에 대응하는 픽셀 X, 조립 후 실측
GRIPPER_PY = 280      # 그리퍼 중심에 대응하는 픽셀 Y, 조립 후 실측
VS_GAIN = 0.3         # Visual Servoing 게인 (낮게 시작)
VS_TOLERANCE = 15     # 수렴 판정 (pixels)
VS_MAX_ITER = 10      # 최대 반복 횟수

# ─── YOLO ────────────────────────────────────────────────────
YOLO_CONF = 0.7
STABLE_FRAMES = 3     # 연속 검출 프레임 수
CLASS_NAMES = {0: "bolt", 1: "nut", 2: "washer"}

# ─── Bins (대략적 J1, 티칭 궤적) ────────────────────────────
BINS = {
    "bolt":   {"j1": 45.0,  "traj": "bolt_place.csv"},
    "nut":    {"j1": 135.0, "traj": "nut_place.csv"},
    "washer": {"j1": 180.0, "traj": "washer_place.csv"},
}

# ─── Trajectories ───────────────────────────────────────────
TRAJECTORY_DIR = "trajectories"
SCAN_TRAJ = "scan_path.csv"

# ─── S-curve Motion ─────────────────────────────────────────
MOVE_DURATION = 1.0
MOVE_DURATION_FAST = 0.3
CONTROL_PERIOD_MS = 20
