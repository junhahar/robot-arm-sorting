# -*- coding: utf-8 -*-
"""ik_pose.py — pose(샤프트 grasp) 파이프라인 전용 IK 래퍼. ik.py는 안 건드림(격리).

IK 수식(solve_ik/solve_pick/check_limits)·관절 길이·너트 오프셋은 **ik.py 그대로 재사용**.
다른 점은 딱 하나 — 테이블→팔 변환 행렬(`homography_to_arm`):

  · 잡는 점이 "박스 중심" → "샤프트"로 바뀌므로 캘리를 새로 떠야 정확해진다.
  · 일단은 ik.py(박스중심 forward-grab) 행렬을 그대로 재사용 → 근사(어느정도 반영).
  · detect_pose.py 로 새 forward-grab 9점을 떠서 H_ARM_POSE 만 교체하면 정확해짐.
    (solve_ik 삼각함수는 안 바뀜 — 바뀌는 건 이 행렬뿐.)

→ ik.py·ik_server.py·OBB 파이프라인 전부 그대로. detect_pose / ik_server_pose 만 이걸 쓴다.
"""
import numpy as np

from ik import (  # IK 수식·상수는 ik.py 단일 소스 재사용
    solve_ik, solve_pick, check_limits, MOTOR_LIMITS,
    GRASP_HEIGHT, NUT_Z_OFFSET, NUT_M6, NUT_DX, NUT_DY,
    H, L1, L2, L3, MAX_REACH, MIN_REACH, M1_OFFSET, M1_GAIN,
)

# pose grasp점(샤프트) 전용 테이블→팔 호모그래피.
# detect_pose forward-grab 10점 중 outlier(C6) 제외 9점으로 재피팅 (RMS 7.0mm).
# (fit_pose_from_armlog.py 참고. 팔 반복정밀도 ~15mm가 바닥이라 그 이하는 의미 없음.)
# ★ 재캘리하려면 fit_pose_from_armlog.py 의 DATA 갱신 후 출력 행렬로 교체.
# [니즈 웹캠 2026-06-18] 11점 forward-grab 재피팅 (RMS 5.9mm, 4-arg M5+L3 tip). OV9782=arducam_original/ik_pose.py
H_ARM_POSE = np.array([
    [-0.20765931, +1.08477773, +113.004564],
    [+0.85698621, +0.04651547, +2.488084],
    [-0.00092377, +0.00066763, +1.000000],
])


# ── 지역 보정 (precise forward-grab 잔차 anchor, Gaussian RBF 보간) ──
# 전역 호모가 가장자리서 systematic 오차 → 그 영역만 보정, 중앙은 →0.
# 좌/우 분리 SIGMA: 우측 앵커(SIGMA=25)가 중앙·좌측으로 스필오버하지 않도록.
# 가중치는 RBF 보간행렬 풀이(측정점서 정확 재현, 음수 정상).
# [니즈 2026-06-18] OV9782 잔차 앵커 비움 — 니즈 글로벌 H_ARM_POSE만으로 시작.
# 실잡기서 특정 구역 systematic 빗나가면 그때 니즈 앵커 추가(((real_x,real_y),(dx,dy))).
_RBF_LEFT = []
_RBF_LEFT_SIGMA = 40.0

_RBF_RIGHT = []
_RBF_RIGHT_SIGMA = 25.0


def _local_offset(hx, hy):
    """좌우 분리 RBF 보정. 좌 SIGMA=40, 우 SIGMA=25 (중앙 스필오버 차단)."""
    ox = oy = 0.0
    for (ax, ay), (dx, dy) in _RBF_LEFT:
        w = np.exp(-((hx - ax) ** 2 + (hy - ay) ** 2) / (2 * _RBF_LEFT_SIGMA ** 2))
        ox += w * dx
        oy += w * dy
    for (ax, ay), (dx, dy) in _RBF_RIGHT:
        w = np.exp(-((hx - ax) ** 2 + (hy - ay) ** 2) / (2 * _RBF_RIGHT_SIGMA ** 2))
        ox += w * dx
        oy += w * dy
    return ox, oy


# ── 추가 구역(ㄴ자서 제외됐던 오른쪽 아래) 보정: 베이스서 +10mm 반경방향 ──
# 기존 왼쪽 RBF와 만나는 왼쪽·상단 변은 10mm 들여(테이블 hx≤140, hy≤51) 합산 간섭 0.
# 가장자리 10mm 페이드. 오른쪽·아래(작업영역 바깥 경계)는 안 들임.
# (코어 구역은 hx>146(왼쪽기둥)·hy>56(위쪽막대)이라 박스 밖 → 자동 보존.)
_ADD_HX_MAX, _ADD_HY_MAX = 140.0, 51.0   # 왼쪽·상단 컷 (원래 150/61서 10mm 들임)
_ADD_PUSH, _ADD_FADE = 0.0, 10.0         # [니즈 2026-06-18] OV9782 보정 끔(0). 니즈 글로벌만으로 시작


def _fade_below(v, vmax, fade):
    """v≤vmax-fade:1, v≥vmax:0, 사이 선형 (경계 부드럽게)."""
    if v <= vmax - fade:
        return 1.0
    if v >= vmax:
        return 0.0
    return (vmax - v) / fade


def _add_weight(hx, hy):
    """추가구역 가중치(0~1): full 1, 경계 페이드, 밖 0."""
    return _fade_below(hx, _ADD_HX_MAX, _ADD_FADE) * _fade_below(hy, _ADD_HY_MAX, _ADD_FADE)


def _add_offset(hx, hy, ax, ay):
    """추가 구역서 베이스 반경방향으로 _ADD_PUSH mm 밀어냄 (박스 안만)."""
    w = _add_weight(hx, hy)
    if w <= 0.0:
        return 0.0, 0.0
    r = (ax * ax + ay * ay) ** 0.5 or 1.0
    return w * _ADD_PUSH * ax / r, w * _ADD_PUSH * ay / r


# ── 좌우(θ1) 보정: 정면은 정확, 좌우 끝에서 ~5mm 덜 감(시차·호모 잔차) → 반경방향 push ──
# 정면(θ1=0)은 0, |θ1|이 _LR_REF_DEG(좌우 끝 근사)면 full _LR_PUSH, 그 사이 선형. 좌우 대칭.
_LR_PUSH = 0.0          # [니즈 2026-06-18] OV9782 보정 끔(0). 니즈 글로벌만으로 시작
_LR_REF_DEG = 35.0      # full push 되는 |θ1|(작업영역 좌우 끝 근사). 끝서 못미치면 ↓, 넘으면 ↑


def _lr_offset(ax, ay):
    """좌우로 갈수록 반경방향으로 _LR_PUSH까지 밀어줌 (정면=0, 좌우 대칭)."""
    th1 = abs(np.degrees(np.arctan2(ay, ax)))
    w = min(th1 / _LR_REF_DEG, 1.0)
    if w <= 0.0:
        return 0.0, 0.0
    r = (ax * ax + ay * ay) ** 0.5 or 1.0
    return w * _LR_PUSH * ax / r, w * _LR_PUSH * ay / r


def homography_to_arm(hx, hy):
    """테이블 mm → 팔 mm (pose grasp 행렬 + 왼쪽 RBF + 추가구역 + 좌우 보정)."""
    p = H_ARM_POSE @ np.array([float(hx), float(hy), 1.0])
    ax, ay = float(p[0] / p[2]), float(p[1] / p[2])
    ox, oy = _local_offset(float(hx), float(hy))
    dx, dy = _add_offset(float(hx), float(hy), ax, ay)
    lx, ly = _lr_offset(ax, ay)
    w_add = _add_weight(float(hx), float(hy))   # 추가구역은 _add가 반경보정 전담 → lr 이중보정 제거(부드럽게 감쇠)
    lx *= (1.0 - w_add)
    ly *= (1.0 - w_add)
    return ax + ox + dx + lx, ay + oy + dy + ly
