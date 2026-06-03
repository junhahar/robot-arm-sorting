#!/usr/bin/env python3
"""
arm_ik.py — 삼보모터스 5축(6모터) 로봇팔 역기구학 (우리 팔 전용 통합본)
================================================================================
설계 결정 (검토의 검토):
  · 코어 IK = '기하학적(해석적) 2링크' 방식 (레포 ik.py 계열).
    - 5축이라 6자유도 완전제약은 불가 → 위치(3) + 접근방향만 잡고 손목회전은 자유.
    - DLS(수치) 대비: 항상 닫힌해, 수렴실패 없음, 빠름, 특이점 안전.
  · 물리 치수/작업구조 = 사용자 DLS 코드(H,D1,L1,L2,L3, 작업높이, 그립자세)에서 가져옴.
  · 모터 변환층 = 우리 STS3215 EEPROM 영점에 맞춤. 모터마다 '영점기준(cal_ref)'을
    따로 둔다:  M2→360, M3→0(거울), M4→360,  M1·M5·M6→180.

좌표계 (base J1 원점, z 위):
  x,y = 수평면, z = 높이(지면 = -H).  r = sqrt(x²+y²) = 수평 도달거리.
  팔 평면(어깨 J2 → 팔꿈치 J3 → 손목 J4 → TCP)을 J1이 회전시킨다.

관절 각(IK 내부, rad):
  J1 = base yaw,  J2 = 어깨(수평 r축 기준),  J3 = 팔꿈치(L1 대비 상대),
  J4 = 손목 피치(L2 대비 상대),  J5 = 손목 롤(위치 무관, 공구 정렬용).

⚠ 실측 필요 상수는 [MEASURE] 로 표시. 실제 팔에서 값 채워야 정확.
"""

import math

# ── 물리 치수 (mm) ─────────────────────────────────────────
H  = 69.3    # 지면 → J1
D1 = 48.5    # J1 → 어깨 J2 (수직)
L1 = 245.0   # 어깨 J2 → 팔꿈치 J3
L2 = 213.0   # 팔꿈치 J3 → 손목 J4
L3 = 87.0    # 손목 J4 → 그리퍼 TCP

# ── 작업 높이 (base 프레임 z) ──────────────────────────────
Z_GROUND       = -H
Z_BOLT_STAND   = Z_GROUND + 10.0
Z_BOLT_LIE     = Z_GROUND + 2.0
Z_PRE_APPROACH = Z_GROUND + 50.0

# ── 관절 한계 (IK 내부 rad) ────────────────────────────────
# [MEASURE] 실제 가동범위로 조정
J_LIMITS = {
    "J1": (math.radians(-120), math.radians(120)),
    "J2": (math.radians(-120), math.radians(120)),
    "J3": (math.radians(-150), math.radians(150)),
    "J4": (math.radians(-120), math.radians(120)),
    "J5": (math.radians(-120), math.radians(120)),
}

EPS = 1e-6


# ============================================================
# 순기구학 (검증·확인용)
# ============================================================
def fk(j1, j2, j3, j4):
    """관절각(rad) → TCP (x, y, z) mm.  j5(롤)는 위치 무관."""
    shoulder = j2
    elbow_w  = j2 + j3           # 팔꿈치 링크의 월드각
    tool_w   = j2 + j3 + j4      # 공구 링크의 월드각

    # 평면 내 (r, z) — 어깨는 (0, D1)에서 시작
    rw = L1 * math.cos(shoulder) + L2 * math.cos(elbow_w)
    zw = D1 + L1 * math.sin(shoulder) + L2 * math.sin(elbow_w)
    r  = rw + L3 * math.cos(tool_w)
    z  = zw + L3 * math.sin(tool_w)

    x = r * math.cos(j1)
    y = r * math.sin(j1)
    return x, y, z


# ============================================================
# 평면 2링크 IK (어깨→손목)
# ============================================================
def _planar_2link(rw, zw, elbow_up=True):
    """어깨(0,D1)에서 손목(rw,zw)까지 L1,L2.  (shoulder, elbow) rad 또는 None."""
    dx = rw - 0.0
    dy = zw - D1
    d  = math.hypot(dx, dy)
    if d > (L1 + L2) - EPS or d < abs(L1 - L2) + EPS:
        return None  # 도달 불가

    gamma = math.atan2(dy, dx)
    cos_a = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
    cos_e = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)
    cos_a = max(-1.0, min(1.0, cos_a))
    cos_e = max(-1.0, min(1.0, cos_e))
    a = math.acos(cos_a)             # 어깨-손목선과 L1 사이각
    e_in = math.acos(cos_e)          # 내부 팔꿈치각

    if elbow_up:
        shoulder = gamma + a
        elbow = -(math.pi - e_in)
    else:
        shoulder = gamma - a
        elbow = (math.pi - e_in)
    return shoulder, elbow


# ============================================================
# 역기구학 (위치 + 접근방향)
# ============================================================
def ik(x, y, z, approach="down", tool_roll=0.0, elbow_up=True):
    """
    목표 TCP (x,y,z) mm + 접근방향 → 관절각 dict(rad) 또는 None.
      approach="down"       : 그리퍼 수직 아래 (세운 볼트/너트)
      approach="horizontal" : 그리퍼 수평 (누운 볼트)
      tool_roll             : J5 손목 롤 (공구를 물체 방향에 정렬, rad)
    5축이라 '위치 + 접근방향'만 잡고 접근축 둘레 회전은 J5로 자유 지정.
    """
    r = math.hypot(x, y)
    j1 = math.atan2(y, x)

    if approach == "down":
        tool_w = -math.pi / 2.0       # 공구 링크가 아래로
        rw = r
        zw = z + L3                   # 손목은 TCP에서 L3 위
    elif approach == "horizontal":
        tool_w = 0.0                  # 공구 링크가 수평(바깥쪽)
        rw = r - L3                   # 손목은 TCP에서 L3 안쪽
        zw = z
    else:
        raise ValueError("approach must be 'down' or 'horizontal'")

    sol = _planar_2link(rw, zw, elbow_up)
    if sol is None:
        return None
    shoulder, elbow = sol
    j4 = tool_w - (shoulder + elbow)  # 손목 피치 = 월드목표 - 팔각

    q = {"J1": j1, "J2": shoulder, "J3": elbow, "J4": j4, "J5": tool_roll}

    # 관절 한계 체크
    for name, val in q.items():
        lo, hi = J_LIMITS[name]
        if not (lo - 1e-9 <= val <= hi + 1e-9):
            return None

    # FK 라운드트립 자체검증 (1mm 이내)
    xc, yc, zc = fk(q["J1"], q["J2"], q["J3"], q["J4"])
    if abs(xc - x) > 1.0 or abs(yc - y) > 1.0 or abs(zc - z) > 1.0:
        return None
    return q


def reachable(x, y, z, approach="down"):
    return ik(x, y, z, approach=approach) is not None


# ============================================================
# 관절각(rad) → STS3215 모터 명령 (우리 영점에 맞춤)
# ============================================================
# 모터 명령각 = cal_ref + DIR × ( deg(joint) − JOINT_DEG_AT_CAL )
#   cal_ref         : 영점 키 누를 때 그 서보가 읽는 각도 (M2=360, M3=0, M4=360, 그외 180)
#   JOINT_DEG_AT_CAL: 그 영점 자세에서의 IK 관절각(deg)  [MEASURE]
#   DIR             : +1 / -1, 관절각↑일 때 서보 읽음↑(+1)/↓(-1)  [MEASURE]
# 거울쌍 J2 → M2,M3 : cal_ref 360/0, DIR 서로 반대.
MOTOR_CAL = {
    # joint   : list of (motor_id, cal_ref_deg, DIR, JOINT_DEG_AT_CAL)
    # DIR 규칙: 360에서 영점잡은 모터는 거기서 '내려가야' 하므로 보통 DIR=-1,
    #           0에서 영점잡은 모터는 '올라가야' 하므로 DIR=+1. (실측으로 부호 확정)
    "J1": [(1, 180.0, +1, 0.0)],                       # [MEASURE] DIR, 기준자세
    "J2": [(2, 360.0, -1, 0.0),                        # 어깨 주모터 (360 영점 → 내려감)
           (3,   0.0, +1, 0.0)],                       # 어깨 거울 (0 영점 → 올라감, 반대 DIR)
    "J3": [(4, 360.0, +1, 0.0)],                       # 팔꿈치 (360 영점) [MEASURE DIR]
    "J4": [(5, 180.0, +1, 0.0)],                       # 손목 피치 (180)
    "J5": [(6, 180.0, +1, 0.0)],                       # 손목 롤 (180)
}

ANGLE_MIN, ANGLE_MAX = 0.0, 360.0


def joints_to_motor_angles(q):
    """관절각 dict(rad) → {motor_id: 명령각(deg, 0~360)}.  STM CMD_SET_ANGLE용."""
    cmd = {}
    for jname, motors in MOTOR_CAL.items():
        jdeg = math.degrees(q[jname])
        for mid, cal_ref, d, jcal in motors:
            a = cal_ref + d * (jdeg - jcal)
            a = max(ANGLE_MIN, min(ANGLE_MAX, a))      # 0~360 clamp (가동범위 밖이면 잘림)
            cmd[mid] = a
    return cmd


# ============================================================
# 볼트/너트 그립 (사전접근 → 최종그립)
# ============================================================
def compute_grip(obj_class, world_xy, bolt_angle=0.0, elbow_up=True):
    """반환: (q_pre, q_grip, ok). 둘 다 IK 성공해야 ok=True."""
    x, y = world_xy
    if obj_class in ("bolt_stand", "nut"):
        z, approach, roll = Z_BOLT_STAND, "down", 0.0
    else:  # bolt_lie
        z, approach, roll = Z_BOLT_LIE, "horizontal", bolt_angle

    q_pre  = ik(x, y, Z_PRE_APPROACH, approach="down", elbow_up=elbow_up)
    q_grip = ik(x, y, z, approach=approach, tool_roll=roll, elbow_up=elbow_up)
    ok = (q_pre is not None) and (q_grip is not None)
    return q_pre, q_grip, ok


# ============================================================
# 자체 검증
# ============================================================
if __name__ == "__main__":
    print("=== FK 라운드트립 검증 ===")
    tests = [
        (300.0,   0.0, -40.0),
        (250.0, 100.0,  50.0),
        (200.0, -80.0, -50.0),
        (350.0,  50.0,   0.0),
    ]
    for (x, y, z) in tests:
        q = ik(x, y, z, approach="down")
        if q is None:
            print(f"  target({x},{y},{z}): 도달 불가")
            continue
        xc, yc, zc = fk(q["J1"], q["J2"], q["J3"], q["J4"])
        err = math.sqrt((xc-x)**2 + (yc-y)**2 + (zc-z)**2)
        deg = {k: round(math.degrees(v), 1) for k, v in q.items()}
        print(f"  target({x:6.1f},{y:6.1f},{z:6.1f}) -> {deg}  | FK오차 {err:.3f}mm")

    print("\n=== 모터 명령 변환 예 (영점기준 적용) ===")
    q = ik(300.0, 0.0, -40.0, approach="down")
    if q:
        print("  관절(deg):", {k: round(math.degrees(v),1) for k,v in q.items()})
        print("  모터명령(deg):", {k: round(v,1) for k,v in joints_to_motor_angles(q).items()})
