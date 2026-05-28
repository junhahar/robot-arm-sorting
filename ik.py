"""
2-Link Geometric IK/FK for 6-Axis MK1 Robot Arm

좌표계:
  r = 수평 거리 (mm, base에서 gripper까지)
  z = 수직 높이 (mm, 양수=위, 음수=아래)
  J1 = base 회전 (xy 평면), J2/J3/J4 = arm 평면 IK
"""
import math
import numpy as np
from config import (
    IK_L2, IK_L3, JOINTS, GRIP_OFFSET_RAD,
    IK_J2_OFFSET, IK_J2_RANGE_DEG,
    IK_J3_OFFSET, IK_J3_RANGE_DEG,
    IK_J4_OFFSET, IK_J4_RANGE_DEG,
)


# ── joint angle <-> internal radian ──

def _a1_to_j2(a1):
    return IK_J2_OFFSET + (a1 + math.pi / 2) * IK_J2_RANGE_DEG / math.pi

def _j2_to_a1(j2):
    return -(math.pi / 2) + (j2 - IK_J2_OFFSET) * math.pi / IK_J2_RANGE_DEG

def _rel3_to_j3(rel3):
    return IK_J3_OFFSET + rel3 * IK_J3_RANGE_DEG / math.pi

def _j3_to_rel3(j3):
    return (j3 - IK_J3_OFFSET) * math.pi / IK_J3_RANGE_DEG

def _rel4_to_j4(rel4):
    return IK_J4_OFFSET + rel4 * IK_J4_RANGE_DEG / math.pi

def _j4_to_rel4(j4):
    return (j4 - IK_J4_OFFSET) * math.pi / IK_J4_RANGE_DEG


def _in_joint_limits(j2, j3, j4):
    if not (JOINTS["J2"]["min"] <= j2 <= JOINTS["J2"]["max"]):
        return False
    if not (JOINTS["J3"]["min"] <= j3 <= JOINTS["J3"]["max"]):
        return False
    if not (JOINTS["J4"]["min"] <= j4 <= JOINTS["J4"]["max"]):
        return False
    return True


def solve(target_r, target_z, elbow_up=True):
    """IK: (r, z) mm -> (J2, J3, J4) degrees, 실패 시 None"""
    target_r = max(target_r, 30.0)
    d = math.sqrt(target_r ** 2 + target_z ** 2)
    d_max = IK_L2 + IK_L3 - 10
    d_min = abs(IK_L2 - IK_L3) + 10

    if d < d_min or d > d_max:
        return None

    gamma = math.atan2(-target_z, target_r)
    cos_e = (IK_L2 ** 2 + IK_L3 ** 2 - d ** 2) / (2 * IK_L2 * IK_L3)
    cos_a = (IK_L2 ** 2 + d ** 2 - IK_L3 ** 2) / (2 * IK_L2 * d)

    cos_e = float(np.clip(cos_e, -1, 1))
    cos_a = float(np.clip(cos_a, -1, 1))

    elbow = math.acos(cos_e)
    alpha = math.acos(cos_a)

    if elbow_up:
        a1 = gamma - alpha
    else:
        a1 = gamma + alpha

    rel3 = math.pi - elbow
    a2 = a1 + rel3
    rel4 = GRIP_OFFSET_RAD - a2

    j2 = _a1_to_j2(a1)
    j3 = _rel3_to_j3(rel3)
    j4 = _rel4_to_j4(rel4)

    if not _in_joint_limits(j2, j3, j4):
        return None

    r_check, z_check = forward(j2, j3, j4)
    if abs(r_check - target_r) > 2.0 or abs(z_check - target_z) > 2.0:
        return None

    return j2, j3, j4


def forward(j2, j3, j4):
    """FK: (J2, J3, J4) degrees -> (r, z) mm"""
    a1 = _j2_to_a1(j2)
    rel3 = _j3_to_rel3(j3)
    a2 = a1 + rel3

    r = IK_L2 * math.cos(a1) + IK_L3 * math.cos(a2)
    z = -(IK_L2 * math.sin(a1) + IK_L3 * math.sin(a2))
    return r, z


def gripper_tilt(j2, j3, j4):
    """그리퍼 기울기(rad) - 수평=0, 수직 아래=-pi/2"""
    a1 = _j2_to_a1(j2)
    rel3 = _j3_to_rel3(j3)
    rel4 = _j4_to_rel4(j4)
    return a1 + rel3 + rel4


def reachable(target_r, target_z):
    """IK 도달 가능 여부"""
    d = math.sqrt(target_r ** 2 + target_z ** 2)
    d_min = abs(IK_L2 - IK_L3) + 10
    d_max = IK_L2 + IK_L3 - 10
    return d_min < d < d_max


def xy_to_polar(x_mm, y_mm):
    """작업 좌표 (x, y) -> (J1 degrees, r mm)"""
    r = math.sqrt(x_mm ** 2 + y_mm ** 2)
    angle_rad = math.atan2(x_mm, y_mm)
    j1 = 90.0 - math.degrees(angle_rad)
    return j1, r
