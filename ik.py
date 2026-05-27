"""
2-Link Geometric IK/FK for 6-Axis MK1 Robot Arm

좌표계:
  r = 수평 거리 (mm, base에서 gripper까지)
  z = 수직 높이 (mm, 양수=위, 음수=아래)
  J1 = base 회전 (xy 평면), J2/J3/J4 = arm 평면 IK
"""
import math
import numpy as np
from config import IK_L2, IK_L3

GRIP_OFFSET = 0.175  # rad, gripper 수직 보정


def solve(target_r, target_z):
    """IK: (r, z) mm → (J2, J3, J4) degrees"""
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
    cos_e = float(np.clip(
        (IK_L2 ** 2 + IK_L3 ** 2 - d ** 2) / (2 * IK_L2 * IK_L3), -1, 1
    ))
    elbow = math.acos(cos_e)
    cos_a = float(np.clip(
        (IK_L2 ** 2 + d ** 2 - IK_L3 ** 2) / (2 * IK_L2 * d), -1, 1
    ))
    alpha = math.acos(cos_a)

    a1 = gamma - alpha
    rel3 = math.pi - elbow
    a2 = a1 + rel3
    rel4 = GRIP_OFFSET - a2

    j2 = float(np.clip(75.0 + (a1 + math.pi / 2) * 150.0 / math.pi, 75, 150))
    j3 = float(np.clip(90.0 + rel3 * 135.0 / math.pi, 0, 170))
    j4 = float(np.clip(90.0 + rel4 * 120.0 / math.pi, 30, 150))
    return j2, j3, j4


def forward(j2, j3, j4):
    """FK: (J2, J3, J4) degrees → (r, z) mm"""
    a1 = -(math.pi / 2) + (j2 - 75) * math.pi / 150
    rel3 = (j3 - 90) * math.pi / 135
    a2 = a1 + rel3

    r = IK_L2 * math.cos(a1) + IK_L3 * math.cos(a2)
    z = -(IK_L2 * math.sin(a1) + IK_L3 * math.sin(a2))
    return r, z


def reachable(target_r, target_z):
    """IK 도달 가능 여부"""
    d = math.sqrt(target_r ** 2 + target_z ** 2)
    d_min = abs(IK_L2 - IK_L3) + 10
    d_max = IK_L2 + IK_L3 - 10
    return d_min < d < d_max


def xy_to_polar(x_mm, y_mm):
    """작업 좌표 (x, y) → (J1 degrees, r mm)
    좌표: +y = 전방(J1=90°), +x = 우측(J1<90°)
    """
    r = math.sqrt(x_mm ** 2 + y_mm ** 2)
    angle_rad = math.atan2(x_mm, y_mm)
    j1 = 90.0 - math.degrees(angle_rad)
    return j1, r
