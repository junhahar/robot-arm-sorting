# -*- coding: utf-8 -*-
"""
ik.py — 로봇 팔 역기구학 (팀원 버전 + 우리 캘리 반영)

반영한 것
  · M2_DIR = -1  (방향 테스트: 어깨 들기 = M2 감소 → M2/M3 한계 안으로)
  · MOTOR_LIMITS + check_limits()  (실측 모터 범위 밖이면 경고)
  · 나머지(M4·M5 OFFSET, 워크스페이스)는 하드웨어 튜닝 필요 — 아래 주석

모터 구성
  M1      : 베이스 회전 (좌/우 yaw)
  M2 + M3 : 어깨 (mirror pair — M3은 M2 반대방향)
  M4      : 팔꿈치 pitch
  M5      : 손목 pitch
  M6      : 손목 roll (볼트 각도 맞춤)
"""
import sys
import numpy as np

sys.stdout.reconfigure(encoding='utf-8')

# ── 링크 길이 (mm) ──────────────────────────
H  = 117
L1 = 245
L2 = 213
L3 = 231
MAX_REACH = L1 + L2
MIN_REACH = abs(L1 - L2)
GRASP_HEIGHT = 23    # 잡기 높이(z). 그리퍼가 평면에 부딪혀서 ↑ (22→23). 너무 높이면 볼트 못 잡으니 부딪힘 없는 최저로
NUT_Z_OFFSET = 2.5    # 너트 grab 높이(GRASP_HEIGHT 위). 4.0은 약간 높아 1.5mm↓ → 작업면에 더 붙여 안정(2~3서 미세조정)
NUT_M6 = 180.0        # 너트는 6각 대칭 → roll 무관, 중립(범위 중앙)으로 고정
NUT_DX, NUT_DY = 10.0, 3.0  # 너트 전용 오프셋(mm). H_ARM은 볼트로 피팅→너트 검출점차이 ~12mm 반경바깥 보정(4점 실측 평균)

# ── 호모그래피 원점 → M1 축 (실측) ──────────
ARM_BASE_X = 285.0
ARM_BASE_Y = 320.0
# 호모그래피 프레임 → 로봇 프레임 회전 (캘리: 정면 볼트 1점, real(252,126))
HOMO_ROT_DEG = 99.6

# ── 모터 캘리브레이션 ───────────────────────
# OFFSET : 중립 자세 모터값, DIR : 회전 방향
# 모터 = OFFSET + GAIN*관절각.  GAIN = 기어비 (5점 수동파지 역산)
M1_OFFSET, M1_GAIN = 174.73, -0.8433  # 6점 재피팅 (rms 1.6°)
M2_OFFSET, M2_GAIN =  35.17, +0.7129  # 6점 재피팅 (rms 1.1°, 게인 0.575→0.71)
M3_OFFSET, M3_GAIN = 324.83, -0.7129  # 어깨 mirror (M2+M3=360 유지)
M4_OFFSET, M4_GAIN =  36.95, -0.9224  # 6점 재피팅 (rms 2.3°)
M5_OFFSET, M5_GAIN = 332.77, +2.3401  # 6점 재피팅 (rms 4.1°, θ4 범위 좁음 주의)
M6_OFFSET, M6_GAIN, M6_TH1_GAIN = 271.45, -1.0528, -0.8385  # roll: bolt·θ1 계수 분리(14점 실측 fit, RMS 5.4°/max 8.5°, ±180 wrap)

# ── 모터 안전 한계 (실측) — IK 출력 검증용 ──
MOTOR_LIMITS = {
    1: (0.0, 360.0),  2: (63.0, 180.0), 3: (180.0, 297.0),
    4: (84.0, 180.0), 5: (130.0, 225.0), 6: (90.0, 270.0),
}


def check_limits(cmd):
    """모터 명령이 실측 한계 밖이면 (mid, val) 리스트 반환 (없으면 빈 리스트)."""
    bad = []
    for mid, val in cmd.items():
        lo, hi = MOTOR_LIMITS.get(mid, (0.0, 360.0))
        if not (lo <= val <= hi):
            bad.append((mid, round(val, 1), (lo, hi)))
    return bad


# 테이블(호모그래피) mm → 로봇 팔 mm : 3x3 perspective homography.
# similarity(4DOF)는 물체 높이 시차를 못 담아 가까운쪽이 틀림 → perspective(8DOF)가 흡수
# (같은 높이 물체의 시차 = 평면-대-평면 투영이라 단일 호모로 표현됨).
# 9점 forward-grab 재피팅 (calib/fit_homography_to_arm.py, 2026-06-07).
# 호모 재촬영 시 보드 이동분 + 근거리 시차를 perspective로 흡수. RMS 9.8mm(이전 44mm).
H_ARM = np.array([
    [+0.05960881, +0.91587667, +66.350798],
    [+0.92016068, +0.03209637, -25.729411],
    [-0.00028381, -0.00124475, +1.000000],
])


def homography_to_arm(hx, hy):
    """테이블(호모그래피) 좌표 mm → 로봇 팔 좌표 mm (perspective)."""
    p = H_ARM @ np.array([float(hx), float(hy), 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def solve_ik(x, y, bolt_angle_deg, z_grasp=GRASP_HEIGHT):
    """목표 (x,y) mm + 볼트각 → {motor_id: 각도} (1~6)."""
    theta1 = np.degrees(np.arctan2(y, x))
    r = np.sqrt(x**2 + y**2)
    r_w = r
    z_w = z_grasp + L3 - H
    D = np.sqrt(r_w**2 + z_w**2)
    if D > MAX_REACH:
        raise ValueError(f"[IK] 도달 불가 — {D:.0f}mm > 최대 {MAX_REACH}mm")
    if D < MIN_REACH:
        raise ValueError(f"[IK] 너무 가까움 — {D:.0f}mm < 최소 {MIN_REACH}mm")
    D = np.clip(D, MIN_REACH + 0.1, MAX_REACH - 0.1)

    gamma = np.degrees(np.arctan2(z_w, r_w))
    cos_delta = np.clip((L1**2 + D**2 - L2**2) / (2 * L1 * D), -1.0, 1.0)
    delta = np.degrees(np.arccos(cos_delta))
    theta2 = gamma + delta

    cos_alpha = np.clip((L1**2 + L2**2 - D**2) / (2 * L1 * L2), -1.0, 1.0)
    alpha = np.degrees(np.arccos(cos_alpha))
    theta3 = alpha - 180.0

    forearm_dir = theta2 + alpha - 180.0
    theta4 = -90.0 - forearm_dir

    # roll: bolt·θ1 계수 분리 (8점 실측 fit). θ1을 bolt보다 약하게(-0.85 vs -1.04).

    cmd = {
        1: M1_OFFSET + M1_GAIN * theta1,
        2: M2_OFFSET + M2_GAIN * theta2,
        3: M3_OFFSET + M3_GAIN * theta2,   # 어깨 mirror (독립 피팅)
        4: M4_OFFSET + M4_GAIN * theta3,
        5: M5_OFFSET + M5_GAIN * theta4,
        6: M6_OFFSET + M6_GAIN * bolt_angle_deg + M6_TH1_GAIN * theta1,
    }
    # M6 roll: 나사축 180° 대칭 → 한계 밖이면 ±180 뒤집어 범위 안으로 (휴리스틱).
    # 한계폭<180일 때 무한루프 방지로 반복 횟수 캡(최대 4회).
    lo6, hi6 = MOTOR_LIMITS[6]
    for _ in range(4):
        if cmd[6] < lo6:
            cmd[6] += 180.0
        elif cmd[6] > hi6:
            cmd[6] -= 180.0
        else:
            break
    for k in cmd:
        cmd[k] = float(np.clip(cmd[k], 0.0, 360.0))
    return cmd


def solve_pick(x, y, bolt_angle_deg, is_bolt):
    """검출 좌표(로봇 arm x,y mm) + 클래스 → 모터 cmd(1~6). 볼트/너트 분기 통합.
    너트: 시차보정(NUT_DX,NUT_DY) + z↑(NUT_Z_OFFSET) + roll 고정(NUT_M6).
    RPi가 이 함수로 IK 계산해 실행 (detect 분기와 같은 상수 사용 → 일관)."""
    if is_bolt:
        gx, gy, gz = x, y, GRASP_HEIGHT
    else:
        gx, gy, gz = x + NUT_DX, y + NUT_DY, GRASP_HEIGHT + NUT_Z_OFFSET
    cmd = solve_ik(gx, gy, bolt_angle_deg, z_grasp=gz)
    if not is_bolt:
        cmd[6] = NUT_M6
    return cmd


if __name__ == "__main__":
    def print_ik(x, y, bolt, z=GRASP_HEIGHT):
        print(f"\n[ x={x:+.0f} y={y:+.0f} z={z} bolt={bolt}° ]")
        try:
            cmd = solve_ik(x, y, bolt, z)
            for mid in sorted(cmd):
                lab = {1:"베이스", 2:"어깨", 3:"어깨mir", 4:"팔꿈치", 5:"손목pit", 6:"손목roll"}
                print(f"  M{mid} {lab[mid]:7s}: {cmd[mid]:6.1f}°")
            bad = check_limits(cmd)
            if bad:
                print("  ⚠ 한계 밖:", [f"M{m}={v}{lim}" for m, v, lim in bad])
            else:
                print("  ✅ 모든 모터 한계 안")
        except ValueError as e:
            print(f"  {e}")

    print("=== IK 테스트 (M2_DIR=-1 반영) ===")
    print(f"최대 도달: {MAX_REACH}mm | 최소: {MIN_REACH}mm")
    print_ik(200, 0, 0)
    print_ik(200, 0, 45)
    print_ik(200, 100, 90)
    print_ik(150, 150, 30)
    print_ik(500, 0, 0)
