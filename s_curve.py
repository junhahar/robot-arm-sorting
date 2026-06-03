"""
S-curve 모션 프로파일 — 급출발/급정지 없는 부드러운 이동
"""
import time
from config import MOVE_DURATION, CONTROL_PERIOD_MS

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5"]


def smoothstep(t, duration):
    """3차 smoothstep: 가속 → 등속 → 감속"""
    x = max(0.0, min(1.0, t / duration))
    return x * x * (3.0 - 2.0 * x)


def interpolate(start, end, ratio):
    """두 자세 사이 선형 보간"""
    return {
        j: start.get(j, 0) + (end.get(j, 0) - start.get(j, 0)) * ratio
        for j in JOINT_NAMES
        if j in start and j in end
    }


def generate(start, end, duration=MOVE_DURATION):
    """S-curve 보간 경로 생성 → [(t_sec, angles_dict), ...]"""
    step_sec = CONTROL_PERIOD_MS / 1000.0
    steps = max(1, int(duration / step_sec))
    profile = []
    for i in range(steps + 1):
        t = i * duration / steps
        ratio = smoothstep(t, duration)
        angles = interpolate(start, end, ratio)
        profile.append((t, angles))
    return profile


def execute(servo, start, end, duration=MOVE_DURATION):
    """S-curve로 start → end 이동 실행"""
    profile = generate(start, end, duration)
    t0 = time.time()
    for t_target, angles in profile:
        elapsed = time.time() - t0
        wait = t_target - elapsed
        if wait > 0:
            time.sleep(wait)
        servo.move_all(angles)
    return end


def execute_j1_only(servo, current, target_j1, duration=MOVE_DURATION):
    """J1만 회전, 나머지 유지"""
    start = dict(current)
    end = dict(current)
    end["J1"] = target_j1
    return execute(servo, start, end, duration)
