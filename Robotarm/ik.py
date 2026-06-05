# -*- coding: utf-8 -*-
"""
ik.py — 로봇 팔 역기구학 (Inverse Kinematics)

모터 구성
  M1      : 베이스 회전 (좌/우 yaw)
  M2 + M3 : 어깨 (mirror pair — M3은 M2 반대방향)
  M4      : 팔꿈치 pitch
  M5      : 손목 pitch (팔꿈치와 동일 축)
  M6      : 손목 roll (볼트 각도 맞춤)

좌표계
  원점 : M1 회전축 (바닥 기준)
  X    : 로봇 전방
  Y    : 로봇 좌측
  Z    : 위 방향
"""
import sys
import numpy as np

sys.stdout.reconfigure(encoding='utf-8')

# ══════════════════════════════════════════
# 링크 길이 (mm)
# ══════════════════════════════════════════
H  = 117   # 바닥  →  어깨 축 (M2/M3)
L1 = 245   # 어깨  →  팔꿈치 (M4)
L2 = 213   # 팔꿈치 →  손목 (M5)
L3 = 231   # 손목  →  그리퍼 끝

# 최대/최소 도달 거리
MAX_REACH = L1 + L2          # 458 mm
MIN_REACH = abs(L1 - L2)     # 32  mm

# 그리퍼 집는 높이 (mm, 바닥 기준) — 실제 테스트 후 조정
GRASP_HEIGHT = 20

# ══════════════════════════════════════════
# 호모그래피 좌표 → 로봇 팔 좌표 변환 오프셋
# 호모그래피 원점(캘리브레이션 기준점)과
# 로봇 M1 회전축 위치 차이 — 실측 후 입력
# ══════════════════════════════════════════
ARM_BASE_X = 285.0  # mm — 호모그래피 원점 → M1 축 X 거리
ARM_BASE_Y = 320.0  # mm — 호모그래피 원점 → M1 축 Y 거리

# ══════════════════════════════════════════
# 모터 캘리브레이션 파라미터
# OFFSET : 중립 자세일 때 모터 명령값 (기본 180°)
# DIR    : 회전 방향 (+1 / -1), 실제 동작 방향이 반대면 -1로 변경
# ══════════════════════════════════════════
M1_OFFSET, M1_DIR = 180.0, +1   # 베이스
M2_OFFSET, M2_DIR = 180.0, +1   # 어깨 (M3 자동 mirror)
M4_OFFSET, M4_DIR = 180.0, +1   # 팔꿈치
M5_OFFSET, M5_DIR = 180.0, +1   # 손목 pitch
M6_OFFSET, M6_DIR = 180.0, +1   # 손목 roll


# ══════════════════════════════════════════
def homography_to_arm(hx: float, hy: float):
    """호모그래피 좌표(mm) → 로봇 팔 좌표(mm) 변환"""
    return hx - ARM_BASE_X, hy - ARM_BASE_Y


def solve_ik(x: float, y: float,
             bolt_angle_deg: float,
             z_grasp: float = GRASP_HEIGHT) -> dict:
    """
    역기구학 계산

    Parameters
    ----------
    x, y           : 물체 실제 좌표 (mm) — 로봇 팔 기준
    bolt_angle_deg : 볼트 각도 0~180° — OpenCV 출력
    z_grasp        : 그리퍼 집는 높이 (mm, 바닥 기준)

    Returns
    -------
    dict {motor_id: angle_command}  motor_id = 1~6, 범위 0~360°
    """

    # ── M1: 베이스 회전 ─────────────────────────
    theta1 = np.degrees(np.arctan2(y, x))

    # ── XY 평면 수평 거리 ────────────────────────
    r = np.sqrt(x**2 + y**2)

    # ── 손목(M5) 목표 위치 ───────────────────────
    # 그리퍼가 수직으로 내려오면 손목은 L3만큼 위에 위치
    r_w = r
    z_w = z_grasp + L3 - H      # 어깨 기준 손목 높이

    # ── 어깨~손목 거리 ───────────────────────────
    D = np.sqrt(r_w**2 + z_w**2)

    # ── 도달 가능 범위 확인 ──────────────────────
    if D > MAX_REACH:
        raise ValueError(f"[IK] 도달 불가 — 거리 {D:.1f}mm > 최대 {MAX_REACH}mm")
    if D < MIN_REACH:
        raise ValueError(f"[IK] 너무 가까움 — 거리 {D:.1f}mm < 최소 {MIN_REACH}mm")
    D = np.clip(D, MIN_REACH + 0.1, MAX_REACH - 0.1)

    # ── 어깨 각도 (M2/M3) ───────────────────────
    # gamma : 어깨에서 손목 방향의 수평 기준 각도
    # delta : law of cosines 로 구한 보정각
    gamma     = np.degrees(np.arctan2(z_w, r_w))
    cos_delta = np.clip((L1**2 + D**2 - L2**2) / (2 * L1 * D), -1.0, 1.0)
    delta     = np.degrees(np.arccos(cos_delta))
    theta2    = gamma + delta          # elbow-up 구성

    # ── 팔꿈치 내각 (M4) ────────────────────────
    cos_alpha = np.clip((L1**2 + L2**2 - D**2) / (2 * L1 * L2), -1.0, 1.0)
    alpha     = np.degrees(np.arccos(cos_alpha))   # 내각 (0°=완전굽힘, 180°=완전폄)
    theta3    = alpha - 180.0          # 완전 폄 기준 편차

    # ── 손목 피치 (M5): 그리퍼 수직 유지 ────────
    forearm_dir = theta2 + alpha - 180.0   # 전완이 수평 기준 이루는 각도
    theta4      = -90.0 - forearm_dir      # 그리퍼가 -90° (수직 하향)가 되도록

    # ── 손목 롤 (M6): 볼트 각도 ─────────────────
    # 볼트 0~180° → 중립(90°) 기준 편차로 변환
    theta5 = bolt_angle_deg - 90.0

    # ── 모터 명령 변환 (중립 180° 기준) ──────────
    cmd = {
        1: M1_OFFSET + M1_DIR * theta1,
        2: M2_OFFSET + M2_DIR * theta2,
        3: M2_OFFSET - M2_DIR * theta2,    # M3: M2 mirror (항상 반대방향)
        4: M4_OFFSET + M4_DIR * theta3,
        5: M5_OFFSET + M5_DIR * theta4,
        6: M6_OFFSET + M6_DIR * theta5,
    }

    # 0~360° 범위 클리핑
    for k in cmd:
        cmd[k] = float(np.clip(cmd[k], 0.0, 360.0))

    return cmd


# ══════════════════════════════════════════
# 테스트 (python ik.py 로 직접 실행)
# ══════════════════════════════════════════
if __name__ == "__main__":

    def print_ik(x, y, bolt, z=GRASP_HEIGHT):
        print(f"\n[ x={x:+.0f}  y={y:+.0f}  z={z}  bolt={bolt}° ]")
        try:
            cmd = solve_ik(x, y, bolt, z)
            for mid in sorted(cmd):
                labels = {1:"베이스  ", 2:"어깨    ", 3:"어깨mir ",
                          4:"팔꿈치  ", 5:"손목pit ", 6:"손목roll"}
                print(f"  M{mid} {labels[mid]}: {cmd[mid]:6.1f}°")
        except ValueError as e:
            print(f"  {e}")

    print("=== IK 테스트 ===")
    print(f"최대 도달 거리: {MAX_REACH}mm  |  최소: {MIN_REACH}mm")

    print_ik(x=200, y=  0, bolt= 0)    # 정면 200mm, 볼트 0도
    print_ik(x=200, y=  0, bolt=45)    # 정면 200mm, 볼트 45도
    print_ik(x=200, y=100, bolt=90)    # 대각선, 볼트 90도
    print_ik(x=150, y=150, bolt=30)    # 대각선
    print_ik(x=500, y=  0, bolt= 0)    # 최대 도달 거리 초과 테스트
