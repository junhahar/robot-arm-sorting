"""
arm_ik.py — 삼보모터스 5축(6모터) 로봇팔 역기구학 (우리 팔 전용 통합본)

설계:
  코어 IK = 기하학적(해석적) 2링크 방식.
  5축이라 위치(3) + 접근방향만 잡고 손목회전은 자유.

좌표계 (base J1 원점, z 위):
  x,y = 수평면, z = 높이(지면 = -H).  r = sqrt(x^2+y^2).

[MEASURE] 표시 = 실측 필요 상수.
"""

import math

# ── 물리 치수 (mm) ─────────────────────────────────────────
H  = 69.3    # 지면 -> J1
D1 = 48.5    # J1 -> 어깨 J2 (수직)
L1 = 245.0   # 어깨 J2 -> 팔꿈치 J3
L2 = 213.0   # 팔꿈치 J3 -> 손목 J4
L3 = 87.0    # 손목 J4 -> 그리퍼 TCP

# ── 작업 높이 (base 프레임 z) ──────────────────────────────
Z_GROUND       = -H
Z_BOLT_STAND   = Z_GROUND + 10.0
Z_BOLT_LIE     = Z_GROUND + 2.0
Z_PRE_APPROACH = Z_GROUND + 50.0

# ── 관절 한계 (IK 내부 rad) ────────────────────────────────
# [MEASURE]
J_LIMITS = {
    "J1": (math.radians(-120), math.radians(120)),
    "J2": (math.radians(-120), math.radians(120)),
    "J3": (math.radians(-150), math.radians(150)),
    "J4": (math.radians(-120), math.radians(120)),
    "J5": (math.radians(-120), math.radians(120)),
}

EPS = 1e-6


# ============================================================
# 순기구학
# ============================================================
def fk(j1, j2, j3, j4):
    """관절각(rad) -> TCP (x, y, z) mm."""
    shoulder = j2
    elbow_w  = j2 + j3
    tool_w   = j2 + j3 + j4

    rw = L1 * math.cos(shoulder) + L2 * math.cos(elbow_w)
    zw = D1 + L1 * math.sin(shoulder) + L2 * math.sin(elbow_w)
    r  = rw + L3 * math.cos(tool_w)
    z  = zw + L3 * math.sin(tool_w)

    x = r * math.cos(j1)
    y = r * math.sin(j1)
    return x, y, z


# ============================================================
# 평면 2링크 IK (어깨->손목)
# ============================================================
def _planar_2link(rw, zw, elbow_up=True):
    dx = rw
    dy = zw - D1
    d  = math.hypot(dx, dy)
    if d > (L1 + L2) - EPS or d < abs(L1 - L2) + EPS:
        return None

    gamma = math.atan2(dy, dx)
    cos_a = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
    cos_e = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)
    cos_a = max(-1.0, min(1.0, cos_a))
    cos_e = max(-1.0, min(1.0, cos_e))
    a = math.acos(cos_a)
    e_in = math.acos(cos_e)

    if elbow_up:
        shoulder = gamma + a
        elbow = -(math.pi - e_in)
    else:
        shoulder = gamma - a
        elbow = (math.pi - e_in)
    return shoulder, elbow


# ============================================================
# 역기구학
# ============================================================
def ik(x, y, z, approach="down", tool_roll=0.0, elbow_up=True):
    """
    목표 TCP (x,y,z) mm + 접근방향 -> 관절각 dict(rad) 또는 None.
      approach="down"       : 그리퍼 수직 아래
      approach="horizontal" : 그리퍼 수평
      tool_roll             : J5 손목 롤 (rad)
    """
    r = math.hypot(x, y)
    j1 = math.atan2(y, x)

    if approach == "down":
        tool_w = -math.pi / 2.0
        rw = r
        zw = z + L3
    elif approach == "horizontal":
        tool_w = 0.0
        rw = r - L3
        zw = z
    else:
        raise ValueError("approach must be 'down' or 'horizontal'")

    sol = _planar_2link(rw, zw, elbow_up)
    if sol is None:
        return None
    shoulder, elbow = sol
    j4 = tool_w - (shoulder + elbow)

    q = {"J1": j1, "J2": shoulder, "J3": elbow, "J4": j4, "J5": tool_roll}

    for name, val in q.items():
        lo, hi = J_LIMITS[name]
        if not (lo - 1e-9 <= val <= hi + 1e-9):
            return None

    xc, yc, zc = fk(q["J1"], q["J2"], q["J3"], q["J4"])
    if abs(xc - x) > 1.0 or abs(yc - y) > 1.0 or abs(zc - z) > 1.0:
        return None
    return q


def reachable(x, y, z, approach="down"):
    return ik(x, y, z, approach=approach) is not None


# ============================================================
# 관절각(rad) -> STS3215 모터 명령각
# ============================================================
# [MEASURE] DIR, JOINT_DEG_AT_CAL
MOTOR_CAL = {
    "J1": [(1, 180.0, +1, 0.0)],
    "J2": [(2, 360.0, -1, 0.0),
           (3,   0.0, +1, 0.0)],
    "J3": [(4, 360.0, +1, 0.0)],
    "J4": [(5, 180.0, +1, 0.0)],
    "J5": [(6, 180.0, +1, 0.0)],
}

ANGLE_MIN, ANGLE_MAX = 0.0, 360.0


def joints_to_motor_angles(q):
    """관절각 dict(rad) -> {motor_id: 명령각(deg, 0~360)}."""
    cmd = {}
    for jname, motors in MOTOR_CAL.items():
        jdeg = math.degrees(q[jname])
        for mid, cal_ref, d, jcal in motors:
            a = cal_ref + d * (jdeg - jcal)
            if a < ANGLE_MIN or a > ANGLE_MAX:
                return None
            cmd[mid] = a
    return cmd


def motor_angles_to_joints(motor_angles):
    """모터 명령각 dict -> 관절각 dict(rad). 역변환 (읽기용)."""
    q = {}
    for jname, motors in MOTOR_CAL.items():
        mid, cal_ref, d, jcal = motors[0]
        if mid in motor_angles:
            jdeg = jcal + (motor_angles[mid] - cal_ref) / d
            q[jname] = math.radians(jdeg)
    return q


# ============================================================
# 그립 계획
# ============================================================
def compute_grip(obj_class, world_xy, bolt_angle=0.0, elbow_up=True):
    """반환: (q_pre, q_grip, ok)."""
    x, y = world_xy
    if obj_class in ("bolt_stand", "nut"):
        z, approach, roll = Z_BOLT_STAND, "down", 0.0
    else:
        z, approach, roll = Z_BOLT_LIE, "horizontal", bolt_angle

    q_pre  = ik(x, y, Z_PRE_APPROACH, approach="down", elbow_up=elbow_up)
    q_grip = ik(x, y, z, approach=approach, tool_roll=roll, elbow_up=elbow_up)
    ok = (q_pre is not None) and (q_grip is not None)
    return q_pre, q_grip, ok


# ============================================================
# 유틸: rad dict <-> deg dict
# ============================================================
def rad_to_deg(q):
    return {k: math.degrees(v) for k, v in q.items()}

def deg_to_rad(q):
    return {k: math.radians(v) for k, v in q.items()}


if __name__ == "__main__":
    print("=== FK round-trip ===")
    tests = [
        (300.0,   0.0, -40.0),
        (250.0, 100.0,  50.0),
        (200.0, -80.0, -50.0),
        (350.0,  50.0,   0.0),
    ]
    for (x, y, z) in tests:
        q = ik(x, y, z, approach="down")
        if q is None:
            print(f"  target({x},{y},{z}): unreachable")
            continue
        xc, yc, zc = fk(q["J1"], q["J2"], q["J3"], q["J4"])
        err = math.sqrt((xc-x)**2 + (yc-y)**2 + (zc-z)**2)
        deg = {k: round(math.degrees(v), 1) for k, v in q.items()}
        print(f"  target({x:6.1f},{y:6.1f},{z:6.1f}) -> {deg}  | err {err:.3f}mm")

    print("\n=== Motor angles ===")
    q = ik(300.0, 0.0, -40.0, approach="down")
    if q:
        print("  joints(deg):", rad_to_deg(q))
        print("  motors(deg):", joints_to_motor_angles(q))
