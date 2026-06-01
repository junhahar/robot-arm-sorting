import cv2
import mediapipe as mp
import numpy as np
import math
import time
import os
import csv
import json
import threading
from config import CARRY

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

FRONT_CAM = 1
SIDE_CAM = 2
SMOOTH_WINDOW = 3
MAX_DELTA = 5.0
DEAD_ZONE = 2.0
MIN_VISIBILITY = 0.7
SAFE_RANGE = 40.0


class LowPassFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.last = None

    def filter(self, val):
        if self.last is None:
            self.last = val
            return val
        self.last += self.alpha * (val - self.last)
        return self.last


class LandmarkSmoother:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.filters = {}

    def smooth(self, cam_id, landmarks):
        if cam_id not in self.filters:
            self.filters[cam_id] = {i: (LowPassFilter(self.alpha), LowPassFilter(self.alpha), LowPassFilter(self.alpha))
                                     for i in range(33)}
        for i, lm in enumerate(landmarks):
            fx, fy, fz = self.filters[cam_id][i]
            lm.x = fx.filter(lm.x)
            lm.y = fy.filter(lm.y)
            lm.z = fz.filter(lm.z)
        return landmarks


class Smoother:
    def __init__(self, window=SMOOTH_WINDOW):
        self.window = window
        self.buffers = {}

    def smooth(self, key, value):
        if key not in self.buffers:
            self.buffers[key] = []
        buf = self.buffers[key]
        buf.append(value)
        if len(buf) > self.window:
            buf.pop(0)
        return sum(buf) / len(buf)


smoother = Smoother()
lm_smoother = LandmarkSmoother(alpha=0.3)

ws_joints = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0, "gripper": "open"}

try:
    import asyncio
    import websockets

    _ws_clients = set()

    async def _ws_handler(websocket, path=None):
        _ws_clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        except Exception:
            pass
        finally:
            _ws_clients.discard(websocket)

    async def _ws_loop():
        server = await websockets.serve(_ws_handler, "localhost", 8765)
        while True:
            if _ws_clients:
                msg = json.dumps(ws_joints)
                for c in list(_ws_clients):
                    try:
                        await c.send(msg)
                    except Exception:
                        _ws_clients.discard(c)
            await asyncio.sleep(0.033)

    def _start_ws():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(_ws_loop())

    threading.Thread(target=_start_ws, daemon=True).start()
    print("[INFO] WebSocket: ws://localhost:8765")
except ImportError:
    print("[WARN] websockets 미설치 - pip install websockets")
except Exception as e:
    print(f"[WARN] WebSocket 실패: {e}")


IK_L2 = 245.0
IK_L3 = 212.0
IK_R_DEF = 322.0
IK_Z_DEF = 285.0
IK_GRIP_A = 0.175


def _ik_2link(target_r, target_z):
    target_r = max(target_r, 30.0)
    d = math.sqrt(target_r ** 2 + target_z ** 2)
    d_max = IK_L2 + IK_L3 - 10
    d_min = abs(IK_L2 - IK_L3) + 10
    d_clamped = float(np.clip(d, d_min, d_max))
    if d > 1e-6 and d_clamped != d:
        s = d_clamped / d
        target_r *= s
        target_z *= s
        d = d_clamped

    gamma = math.atan2(-target_z, target_r)
    cos_e = float(np.clip((IK_L2 ** 2 + IK_L3 ** 2 - d ** 2) / (2 * IK_L2 * IK_L3), -1, 1))
    elbow_int = math.acos(cos_e)
    cos_a = float(np.clip((IK_L2 ** 2 + d ** 2 - IK_L3 ** 2) / (2 * IK_L2 * d), -1, 1))
    alpha = math.acos(cos_a)

    a1 = gamma - alpha
    rel3 = math.pi - elbow_int
    a2 = a1 + rel3
    rel4 = IK_GRIP_A - a2

    j2 = float(np.clip(75.0 + (a1 + math.pi / 2) * 150.0 / math.pi, 75, 150))
    j3 = float(np.clip(90.0 + rel3 * 135.0 / math.pi, 0, 170))
    j4 = float(np.clip(90.0 + rel4 * 120.0 / math.pi, 30, 150))
    return j2, j3, j4


def solve_ik(nx, dny, delbow):
    """기하학 IK.
    nx: 좌우 delta (-1~1)
    dny: 상하 delta (양수=손 내림)
    delbow: 팔꿈치 각도 delta (도)
    """
    j1 = float(np.clip(90 + nx * 80, 0, 180))
    target_r = IK_R_DEF + delbow * 3.0
    target_z = IK_Z_DEF - dny * 350
    j2, j3, j4 = _ik_2link(target_r, target_z)
    j5 = 90.0
    return j1, j2, j3, j4, j5


LINK_LENGTHS = [24, 148, 128]  # 실측 40.5:245:212mm 비례 (×0.6 스케일)
VIEW_AZIM = math.radians(35)
VIEW_ELEV = math.radians(25)


def project_3d(pt, center=(250, 380)):
    x, y, z = pt
    ca, sa = math.cos(VIEW_AZIM), math.sin(VIEW_AZIM)
    rx = x * ca + z * sa
    rz = -x * sa + z * ca
    ce, se = math.cos(VIEW_ELEV), math.sin(VIEW_ELEV)
    ry = y * ce - rz * se
    return (int(center[0] + rx), int(center[1] - ry))


def draw_robot_arm(robot):
    canvas = np.zeros((500, 600, 3), dtype=np.uint8)

    j1 = math.radians(robot["J1"])
    a1 = -(math.pi / 2) + (robot["J2"] - 75) * math.pi / 150
    a2 = a1 + (robot["J3"] - 90) * math.pi / 135
    a3 = a2 + (robot["J4"] - 90) * math.pi / 120

    r, h = 0.0, 0.0
    joints_rh = [(0.0, 0.0)]
    for length, angle in zip(LINK_LENGTHS, [a1, a2, a3]):
        r += length * math.cos(angle)
        h += -length * math.sin(angle)
        joints_rh.append((r, h))

    joints_3d = []
    for rad, height in joints_rh:
        joints_3d.append((rad * math.sin(j1), height, rad * math.cos(j1)))

    grip_angles = [a3 + 0.4, a3 - 0.4]
    for ga in grip_angles:
        gr = joints_rh[3][0] + 20 * math.cos(ga)
        gh = joints_rh[3][1] + (-20 * math.sin(ga))
        joints_3d.append((gr * math.sin(j1), gh, gr * math.cos(j1)))

    pts = [project_3d(j) for j in joints_3d]

    n_ellipse = 32
    base_ring = []
    for i in range(n_ellipse):
        t = 2 * math.pi * i / n_ellipse
        bx = 30 * math.cos(t)
        bz = 30 * math.sin(t)
        base_ring.append(project_3d((bx, -5, bz)))
    cv2.fillPoly(canvas, [np.array(base_ring)], (60, 60, 60))
    cv2.polylines(canvas, [np.array(base_ring)], True, (100, 100, 100), 2)

    j1_arrow = project_3d((25 * math.sin(j1), 0, 25 * math.cos(j1)))
    base_pt = project_3d((0, 0, 0))
    cv2.arrowedLine(canvas, base_pt, j1_arrow, (0, 200, 255), 2, tipLength=0.3)

    colors = [(0, 200, 255), (0, 255, 200), (200, 255, 0)]
    for i in range(3):
        cv2.line(canvas, pts[i], pts[i + 1], colors[i], 7)

    cv2.line(canvas, pts[3], pts[4], (255, 100, 100), 3)
    cv2.line(canvas, pts[3], pts[5], (255, 100, 100), 3)

    cv2.circle(canvas, pts[0], 7, (255, 255, 255), -1)
    for i in range(1, 4):
        cv2.circle(canvas, pts[i], 6, (0, 0, 255), -1)

    labels = ["Base", "Shoulder", "Elbow", "Wrist"]
    for i, label in enumerate(labels):
        cv2.putText(canvas, label, (pts[i][0] + 10, pts[i][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

    for i, (joint, val) in enumerate(robot.items()):
        cv2.putText(canvas, f"{joint}: {val:.1f}", (10, 25 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    j5_center = (540, 80)
    j5_r = 30
    cv2.circle(canvas, j5_center, j5_r, (80, 80, 80), 2)
    cv2.putText(canvas, "J5 Roll", (505, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    j5_rad = math.radians(robot["J5"])
    j5_tip = (int(j5_center[0] + j5_r * math.cos(j5_rad)),
              int(j5_center[1] - j5_r * math.sin(j5_rad)))
    cv2.line(canvas, j5_center, j5_tip, (200, 255, 0), 3)
    cv2.circle(canvas, j5_center, 4, (255, 255, 255), -1)

    return canvas


cap_front = cv2.VideoCapture(FRONT_CAM)
cap_side = cv2.VideoCapture(SIDE_CAM)

print(f"[INFO] 정면({FRONT_CAM}): {'열림' if cap_front.isOpened() else '실패'}")
print(f"[INFO] 측면({SIDE_CAM}): {'열림' if cap_side.isOpened() else '실패'}")

if not cap_front.isOpened():
    print("[ERROR] 정면 카메라 열기 실패")
    cap_front.release()
    cap_side.release()
    exit()
if cap_side.isOpened():
    import queue as _q
    _rq = _q.Queue()
    _t = threading.Thread(target=lambda: _rq.put(cap_side.read()), daemon=True)
    _t.start()
    _t.join(timeout=3.0)
    if _t.is_alive() or _rq.empty() or not _rq.get_nowait()[0]:
        print("[WARN] 측면 카메라 읽기 실패 - 정면만 사용")
        cap_side.release()
else:
    print("[WARN] 측면 카메라 없음 - 정면만 사용")

pose_front = mp_pose.Pose(model_complexity=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
pose_side = mp_pose.Pose(model_complexity=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
print("[INFO] 종료: q")

prev_robot = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0}
baseline = None
side_use_right = False

TRAJECTORY_DIR = "trajectories"
os.makedirs(TRAJECTORY_DIR, exist_ok=True)
recording = False
rec_start = 0.0
rec_data = []
gripper_open = True

AUTO_CALIB_FRAMES = 30
auto_buf = []
auto_base = None
front_arm_right = False
arm_locked = False
joint_lock = {"J1": False, "J2": False, "J3": False, "J4": False, "J5": False}
traj_name = None

print("[INFO] 'c': 캘리브레이션 / 's': 녹화 / 'g': 그리퍼 / 'r': 리셋 / '1-5': 관절잠금 / 'q': 종료")

for _ in range(10):
    cap_front.read()
    if cap_side.isOpened():
        cap_side.read()
    time.sleep(0.1)
print("[INFO] 카메라 워밍업 완료")

while cap_front.isOpened():
    ret1, frame_front = cap_front.read()
    ret2, frame_side = cap_side.read() if cap_side.isOpened() else (False, None)
    if not ret1:
        continue
    if not ret2:
        frame_side = np.zeros_like(frame_front)
        cv2.putText(frame_side, "SIDE CAM OFF", (50, frame_front.shape[0] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    frame_front.flags.writeable = False
    res_front = pose_front.process(cv2.cvtColor(frame_front, cv2.COLOR_BGR2RGB))
    frame_front.flags.writeable = True
    if ret2:
        frame_side.flags.writeable = False
        res_side = pose_side.process(cv2.cvtColor(frame_side, cv2.COLOR_BGR2RGB))
        frame_side.flags.writeable = True
    else:
        res_side = type('obj', (object,), {'pose_landmarks': None})()

    frame_front = cv2.flip(frame_front, 1)
    frame_side = cv2.flip(frame_side, 1)

    robot = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0}

    lm_front = None
    lm_side = None
    if res_front.pose_landmarks:
        lm_front = lm_smoother.smooth("front", res_front.pose_landmarks.landmark)
        if lm_front[11].visibility < 0.5 or lm_front[12].visibility < 0.5:
            lm_front = None
        elif lm_front[15].visibility < 0.5 and lm_front[16].visibility < 0.5:
            lm_front = None
        else:
            if not arm_locked:
                mid = (lm_front[11].x + lm_front[12].x) / 2
                l_d = abs(lm_front[15].x - mid) if lm_front[15].visibility > 0.5 else 0
                r_d = abs(lm_front[16].x - mid) if lm_front[16].visibility > 0.5 else 0
                front_arm_right = r_d > l_d
    if res_side.pose_landmarks:
        lm_side = lm_smoother.smooth("side", res_side.pose_landmarks.landmark)
        _si, _wi = (12, 16) if front_arm_right else (11, 15)
        if lm_side[_si].visibility < 0.5 or lm_side[_wi].visibility < 0.5:
            lm_side = None

    nx = 0.0
    ny = 0.0
    extension = 0.5
    has_tracking = False

    if lm_front is not None:
        mid_x = (lm_front[11].x + lm_front[12].x) / 2
        sw = abs(lm_front[12].x - lm_front[11].x)
        _wrist = lm_front[16] if front_arm_right else lm_front[15]
        nx = (_wrist.x - mid_x) / max(sw * 2, 0.05)
        has_tracking = True

    elbow_deg = 90.0
    if lm_side is not None:
        _s, _e, _w = (12, 14, 16) if front_arm_right else (11, 13, 15)
        s_s, e_s, w_s = lm_side[_s], lm_side[_e], lm_side[_w]
        upper_s = math.sqrt((e_s.x - s_s.x) ** 2 + (e_s.y - s_s.y) ** 2)
        forearm_s = math.sqrt((w_s.x - e_s.x) ** 2 + (w_s.y - e_s.y) ** 2)
        reach_s = upper_s + forearm_s
        sy = w_s.y - s_s.y
        ny = sy / max(reach_s, 0.05)
        v1 = np.array([s_s.x - e_s.x, s_s.y - e_s.y])
        v2 = np.array([w_s.x - e_s.x, w_s.y - e_s.y])
        cos_e = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8)
        elbow_deg = math.degrees(math.acos(np.clip(cos_e, -1.0, 1.0)))
        has_tracking = True
    elif lm_front is not None:
        _s, _e, _w = (12, 14, 16) if front_arm_right else (11, 13, 15)
        s_f, e_f, w_f = lm_front[_s], lm_front[_e], lm_front[_w]
        reach_f = math.sqrt((e_f.x - s_f.x) ** 2 + (e_f.y - s_f.y) ** 2) + \
                  math.sqrt((w_f.x - e_f.x) ** 2 + (w_f.y - e_f.y) ** 2)
        fy = w_f.y - s_f.y
        ny = fy / max(reach_f, 0.05)
        v1 = np.array([s_f.x - e_f.x, s_f.y - e_f.y])
        v2 = np.array([w_f.x - e_f.x, w_f.y - e_f.y])
        cos_e = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8)
        elbow_deg = math.degrees(math.acos(np.clip(cos_e, -1.0, 1.0)))

    if has_tracking:
        if auto_base is None:
            auto_buf.append((nx, ny, elbow_deg))
            if len(auto_buf) >= AUTO_CALIB_FRAMES:
                auto_base = {
                    "nx": np.mean([b[0] for b in auto_buf]),
                    "ny": np.mean([b[1] for b in auto_buf]),
                    "elbow": np.mean([b[2] for b in auto_buf]),
                }
                arm_locked = True
                print(f"[AUTO-CALIB] 기준: nx={auto_base['nx']:.2f} ny={auto_base['ny']:.2f} elbow={auto_base['elbow']:.0f} arm={'R' if front_arm_right else 'L'}")
            else:
                cv2.putText(frame_front, f"AUTO-CALIB {len(auto_buf)}/{AUTO_CALIB_FRAMES}",
                            (10, frame_front.shape[0] - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if auto_base is not None:
            dnx = nx - auto_base["nx"]
            dny = ny - auto_base["ny"]
            delbow = elbow_deg - auto_base["elbow"]
            j1, j2, j3, j4, j5 = solve_ik(dnx, dny, delbow)
            if not joint_lock["J1"]: robot["J1"] = j1
            if not joint_lock["J2"]: robot["J2"] = j2
            if not joint_lock["J3"]: robot["J3"] = j3
            if not joint_lock["J4"]: robot["J4"] = j4

        if lm_front is not None:
            lm_f = lm_front
            _pk, _ix = (18, 20) if front_arm_right else (17, 19)
            if lm_f[_pk].visibility > 0.7 and lm_f[_ix].visibility > 0.7:
                roll_vec = np.array([lm_f[_ix].x - lm_f[_pk].x, lm_f[_ix].y - lm_f[_pk].y])
                vertical = np.array([0, 1])
                cos_val = np.dot(roll_vec, vertical) / (np.linalg.norm(roll_vec) + 1e-8)
                roll_angle = math.degrees(math.acos(np.clip(cos_val, -1.0, 1.0)))
                if abs(roll_angle - 90) > 15 and not joint_lock["J5"]:
                    robot["J5"] = float(np.clip(roll_angle, 0, 180))

    if has_tracking:
        for cam_frame, res in [("front", res_front), ("side", res_side)]:
            if res and res.pose_landmarks:
                frame = frame_front if cam_frame == "front" else frame_side
                lm_draw = res.pose_landmarks.landmark
                fh, fw = frame.shape[:2]
                draw_ids = [12, 14, 16, 18, 20] if front_arm_right else [11, 13, 15, 17, 19]
                pts_draw = {}
                for idx in draw_ids:
                    if lm_draw[idx].visibility > 0.3:
                        px = int((1.0 - lm_draw[idx].x) * fw)
                        py = int(lm_draw[idx].y * fh)
                        pts_draw[idx] = (px, py)
                        cv2.circle(frame, (px, py), 6, (0, 0, 255), -1)
                _conns = [(12, 14), (14, 16), (16, 20), (18, 20)] if front_arm_right else [(11, 13), (13, 15), (15, 19), (17, 19)]
                for a, b in _conns:
                    if a in pts_draw and b in pts_draw:
                        cv2.line(frame, pts_draw[a], pts_draw[b], (0, 255, 0), 3)
        arm_label = "R-ARM" if front_arm_right else "L-ARM"
        cv2.putText(frame_front, arm_label, (frame_front.shape[1] - 100, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        tr = IK_R_DEF + (elbow_deg - auto_base["elbow"] if auto_base else 0) * 2.0
        tz = IK_Z_DEF - (ny - auto_base["ny"] if auto_base else 0) * 250
        cv2.putText(frame_front, f"r:{tr:.0f} z:{tz:.0f} nx:{nx:.2f}",
                    (10, frame_front.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    raw_robot = dict(robot)

    for joint in robot:
        robot[joint] = smoother.smooth(joint, robot[joint])

    DEFAULT_POSE = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0}
    if baseline is not None:
        for joint in robot:
            robot[joint] = DEFAULT_POSE[joint] + (robot[joint] - baseline[joint])
            robot[joint] = float(np.clip(robot[joint], DEFAULT_POSE[joint] - SAFE_RANGE, DEFAULT_POSE[joint] + SAFE_RANGE))
    else:
        for joint in robot:
            robot[joint] = float(np.clip(robot[joint], 0, 180))

    prev_robot = dict(robot)
    ws_joints.update(robot)
    ws_joints["gripper"] = "open" if gripper_open else "close"

    if recording:
        elapsed = int((time.time() - rec_start) * 1000)
        row = {"time_ms": elapsed}
        row.update(robot)
        row["gripper"] = "open" if gripper_open else "close"
        rec_data.append(row)

    calib_text = "CALIB" if baseline else "RAW"
    for i, (joint, val) in enumerate(robot.items()):
        near_limit = baseline is not None and (val <= 90 - SAFE_RANGE + 5 or val >= 90 + SAFE_RANGE - 5)
        is_locked = joint_lock.get(joint, False)
        color = (128, 128, 128) if is_locked else (0, 0, 255) if near_limit else (0, 255, 0)
        text = f"{joint}: {val:.1f}"
        if is_locked:
            text += " L"
        elif near_limit:
            text += " !"
        cv2.putText(frame_front, text, (10, 30 + i * 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame_side, text, (10, 30 + i * 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(frame_front, calib_text, (frame_front.shape[1] - 100, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255) if baseline else (0, 0, 255), 2)
    grip_text = "GRIP: OPEN" if gripper_open else "GRIP: CLOSE"
    grip_color = (0, 255, 0) if gripper_open else (0, 0, 255)
    cv2.putText(frame_front, grip_text, (frame_front.shape[1] - 180, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, grip_color, 2)
    if recording:
        elapsed_sec = (time.time() - rec_start)
        cv2.putText(frame_front, f"REC {elapsed_sec:.1f}s ({len(rec_data)}f)",
                    (frame_front.shape[1] - 250, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    robot_canvas = draw_robot_arm(robot)

    cv2.imshow("Front Camera", frame_front)
    cv2.imshow("Side Camera", frame_side)
    cv2.imshow("Robot Arm", robot_canvas)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        baseline = dict(raw_robot)
        print(f"[CALIB] 기준 자세 저장: {baseline}")
    elif key == ord('r'):
        baseline = None
        auto_base = None
        auto_buf.clear()
        arm_locked = False
        print("[CALIB] 리셋 - RAW 모드 (팔 잠금 해제)")
    elif key == ord('g'):
        gripper_open = not gripper_open
        state = "OPEN" if gripper_open else "CLOSE"
        print(f"[GRIP] 그리퍼 {state}")
    elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
        jname = f"J{key - ord('0')}"
        joint_lock[jname] = not joint_lock[jname]
        state = "LOCK" if joint_lock[jname] else "FREE"
        locked = [k for k, v in joint_lock.items() if v]
        print(f"[LOCK] {jname} {state} | 잠금: {locked if locked else '없음'}")
    elif key == ord('s'):
        if not recording:
            recording = True
            rec_start = time.time()
            rec_data = [{"time_ms": 0, **CARRY, "gripper": "open" if gripper_open else "close"}]
            print("[REC] 녹화 시작 — CARRY 자세 기준 (첫 프레임 자동 삽입)")
            print(f"[REC] CARRY: {CARRY}")
        else:
            recording = False
            if rec_data:
                try:
                    traj_name = input("[REC] 궤적 이름 (예: bolt_place): ").strip()
                except EOFError:
                    traj_name = ""
                if not traj_name:
                    traj_name = f"traj_{int(time.time())}"
                fname = f"{traj_name}.csv"
                fpath = os.path.join(TRAJECTORY_DIR, fname)
                keys = rec_data[0].keys()
                with open(fpath, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=keys)
                    writer.writeheader()
                    writer.writerows(rec_data)
                print(f"[REC] 저장 완료: {fpath} ({len(rec_data)} 프레임)")
            else:
                print("[REC] 데이터 없음")
    elif key == ord('p'):
        traj_files = sorted(
            [f for f in os.listdir(TRAJECTORY_DIR) if f.endswith(".csv")],
            key=lambda f: os.path.getmtime(os.path.join(TRAJECTORY_DIR, f)),
            reverse=True,
        )
        if not traj_files:
            print("[PLAY] 저장된 궤적 없음")
        else:
            play_path = os.path.join(TRAJECTORY_DIR, traj_files[0])
            print(f"[PLAY] 재생: {play_path}")
            with open(play_path, "r") as pf:
                reader = csv.DictReader(pf)
                play_data = list(reader)
            play_smoother = Smoother(window=5)
            play_lpf = LandmarkSmoother(alpha=0.2)
            play_prev = {j: float(play_data[0][j]) for j in ["J1", "J2", "J3", "J4", "J5"]}
            play_start = time.time()
            for row in play_data:
                t_target = float(row["time_ms"]) / 1000.0
                while time.time() - play_start < t_target:
                    cv2.waitKey(1)
                p_robot = {j: float(row[j]) for j in ["J1", "J2", "J3", "J4", "J5"]}
                for j in p_robot:
                    p_robot[j] = play_smoother.smooth(j, p_robot[j])
                    delta = p_robot[j] - play_prev[j]
                    if abs(delta) > MAX_DELTA:
                        p_robot[j] = play_prev[j] + MAX_DELTA * np.sign(delta)
                play_prev = dict(p_robot)
                p_grip = row.get("gripper", "open")
                canvas = draw_robot_arm(p_robot)
                grip_text = "GRIP: OPEN" if p_grip == "open" else "GRIP: CLOSE"
                grip_color = (0, 255, 0) if p_grip == "open" else (0, 0, 255)
                cv2.putText(canvas, grip_text, (10, 170),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, grip_color, 2)
                elapsed = float(row["time_ms"]) / 1000.0
                cv2.putText(canvas, f"PLAY {elapsed:.1f}s", (10, 195),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.imshow("Robot Arm", canvas)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            print("[PLAY] 재생 완료")

cap_front.release()
cap_side.release()
cv2.destroyAllWindows()
