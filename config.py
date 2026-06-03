# ─── Named Poses (IK joint degrees) ────────────────────────
# [MEASURE] 실측 후 교체. arm_ik 관절각 기준(deg).
HOME = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0, "J5": 0.0}

# ─── CAN Bus ───────────────────────────────────────────────
CAN_CHANNEL = "can0"
CAN_BUSTYPE = "socketcan"
CAN_BITRATE = 125000

# ─── Gripper (SG90) ─────────────────────────────────────────
GRIPPER_OPEN = 90
GRIPPER_CLOSE = 30
GRIP_CLOSE_WAIT = 0.5  # seconds

# ─── Work Surface ───────────────────────────────────────────
# arm_ik 좌표계: z=0 = base, z=-H = 지면
GRASP_DIST = 30       # mm — ToF 이 거리 이하면 그리퍼 닫기
PLACE_DIST = 15       # mm — bin 내부 놓기 거리
DESCEND_STEP_MM = 5.0
LIFT_HEIGHT = 40.0    # mm — 파지 후 들어올리기
MAX_PLACE_DEPTH = 120.0  # mm — bin 최대 하강 깊이
MAX_DESCEND_DEPTH = 200.0  # mm — 작업대 최대 하강

# ─── ArUco ──────────────────────────────────────────────────
ARUCO_DICT = "DICT_4X4_50"
BIN_ARUCO = {
    "bolt":   {"id": 1, "offset_x": 0.0, "offset_y": 50.0},
    "nut":    {"id": 2, "offset_x": 0.0, "offset_y": 50.0},
    "washer": {"id": 3, "offset_x": 0.0, "offset_y": 50.0},
}

# ─── Camera (eye-in-hand) ──────────────────────────────────
CAMERA_ID = 0
# [MEASURE] 체커보드 캘리브레이션 후 교체
CAM_FX = 600.0
CAM_FY = 600.0
CAM_CX = 320.0
CAM_CY = 240.0
# [MEASURE] 카메라 마운트 오프셋 (그리퍼 기준, mm)
CAM_OFFSET_X = 30.0
CAM_OFFSET_Y = 0.0

# ─── Visual Servoing ────────────────────────────────────────
GRIPPER_PX = 350      # [MEASURE] 그리퍼 중심 픽셀 X
GRIPPER_PY = 280      # [MEASURE] 그리퍼 중심 픽셀 Y
VS_GAIN = 0.3
VS_TOLERANCE = 15     # pixels
VS_MAX_ITER = 10

# ─── YOLO ────────────────────────────────────────────────────
YOLO_CONF = 0.7
STABLE_FRAMES = 3
CLASS_NAMES = {0: "bolt", 1: "nut", 2: "washer"}

# ─── Bins (티칭 궤적) ──────────────────────────────────────
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
