# -*- coding: utf-8 -*-
"""
Small, file-based vision state bridge for the Sambo dashboard.

Input -> processing -> output:
camera detector writes one JSON state -> this module normalizes it -> dashboard
WebSocket payload can include a safe read-only ``vision`` field.

This module never sends motor commands. It only shares the latest detected
object position, confidence, bbox, and optional IK preview values.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional


DEFAULT_TTL_S = float(os.environ.get("SAMBO_VISION_STATE_TTL_S", "2.5"))


def _default_state_path() -> Path:
    env_path = os.environ.get("SAMBO_VISION_STATE_PATH")
    if env_path:
        return Path(env_path).expanduser()

    here = Path(__file__).resolve().parent
    if here.name.lower() == "vision":
        sibling_dashboard = here.parent / "dashboard"
        if sibling_dashboard.exists():
            return sibling_dashboard / "sambo_vision_state.json"
    return here / "sambo_vision_state.json"


STATE_PATH = _default_state_path()


def empty_vision_state(reason: str = "no_object") -> Dict[str, Any]:
    return {
        "target": "NONE",
        "confidence": 0.0,
        "stable_frames": 0,
        "bbox": None,
        "correction": {"dx_mm": 0.0, "dy_mm": 0.0},
        "source": "vision_state_file",
        "fresh": False,
        "reason": reason,
    }


def empty_gripper_state(reason: str = "missing") -> Dict[str, Any]:
    return {
        "state": "OPEN",
        "sg90_angle": 60.0,
        "source": "runtime_state_file",
        "fresh": False,
        "reason": reason,
    }


def _number(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if result != result:
        return default
    return result


def _first_number(data: Mapping[str, Any], *keys: str) -> Optional[float]:
    for key in keys:
        value = _number(data.get(key))
        if value is not None:
            return value
    return None


def _target_name(value: Any) -> str:
    raw = str(value or "").strip()
    if not raw:
        return "NONE"
    low = raw.lower()
    if "bolt" in low or "볼트" in raw:
        return "BOLT"
    if "nut" in low or "너트" in raw:
        return "NUT"
    if "washer" in low or "와셔" in raw:
        return "WASHER"
    if low in {"none", "wait", "no_object", "background"}:
        return "NONE"
    return raw.upper().replace(" ", "_")


def _point_from(data: Mapping[str, Any]) -> Dict[str, float]:
    x = _first_number(data, "x_mm", "xa", "x_arm", "arm_x_mm", "world_x_mm", "table_x_mm", "pick_x_mm")
    y = _first_number(data, "y_mm", "ya", "y_arm", "arm_y_mm", "world_y_mm", "table_y_mm", "pick_y_mm")
    z = _first_number(data, "z_mm", "height_mm", "arm_z_mm", "world_z_mm", "grasp_z_mm") or 0.0
    if x is not None and y is not None:
        return {"x_mm": x, "y_mm": y, "z_mm": z}

    for key in ("position_mm", "arm_mm", "world_mm", "table_mm", "pick_mm", "point_mm"):
        point = data.get(key)
        if isinstance(point, Mapping):
            px = _first_number(point, "x_mm", "x")
            py = _first_number(point, "y_mm", "y")
            pz = _first_number(point, "z_mm", "z", "height") or 0.0
            if px is not None and py is not None:
                return {"x_mm": px, "y_mm": py, "z_mm": pz}
        elif isinstance(point, (list, tuple)) and len(point) >= 2:
            px = _number(point[0])
            py = _number(point[1])
            pz = _number(point[2], 0.0) if len(point) >= 3 else 0.0
            if px is not None and py is not None:
                return {"x_mm": px, "y_mm": py, "z_mm": pz or 0.0}
    return {}


def normalize_vision_state(payload: Mapping[str, Any], *, now: Optional[float] = None) -> Dict[str, Any]:
    now = time.time() if now is None else now
    data = payload.get("vision") if isinstance(payload.get("vision"), Mapping) else payload
    target = _target_name(data.get("target") or data.get("label") or data.get("class_name") or data.get("class"))
    confidence = max(0.0, min(1.0, _number(data.get("confidence", data.get("conf", data.get("score"))), 0.0) or 0.0))
    stable_frames = int(_number(data.get("stable_frames", data.get("stable", 0)), 0) or 0)
    correction = data.get("correction") if isinstance(data.get("correction"), Mapping) else {}
    result = {
        "target": target,
        "confidence": confidence,
        "stable_frames": stable_frames,
        "bbox": data.get("bbox"),
        "correction": {
            "dx_mm": _number(correction.get("dx_mm", correction.get("dx")), 0.0) or 0.0,
            "dy_mm": _number(correction.get("dy_mm", correction.get("dy")), 0.0) or 0.0,
        },
        "source": data.get("source", "vision_state_file"),
        "fresh": target != "NONE" and confidence > 0.0,
        "updated_at": _number(data.get("updated_at", data.get("timestamp", data.get("ts"))), now) or now,
    }
    result.update(_point_from(data))

    angle = _number(data.get("angle_deg", data.get("bolt_angle_deg", data.get("ba"))))
    if angle is not None:
        result["angle_deg"] = angle
    distance = _number(data.get("distance_mm", data.get("D")))
    if distance is not None:
        result["distance_mm"] = distance
    ik = data.get("ik") or data.get("cmd")
    if isinstance(ik, Mapping):
        result["ik"] = {str(k): _number(v, v) for k, v in ik.items()}
    return result


def load_latest_vision_state(path: Optional[os.PathLike[str] | str] = None,
                             ttl_s: float = DEFAULT_TTL_S) -> Dict[str, Any]:
    state_path = Path(path) if path is not None else STATE_PATH
    try:
        stat = state_path.stat()
    except FileNotFoundError:
        return empty_vision_state("missing")
    except OSError as exc:
        state = empty_vision_state("stat_error")
        state["error"] = str(exc)
        return state

    now = time.time()
    age_s = now - stat.st_mtime
    if age_s > ttl_s:
        state = empty_vision_state("stale")
        state["age_ms"] = int(age_s * 1000)
        return state

    try:
        with state_path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except Exception as exc:
        state = empty_vision_state("read_error")
        state["error"] = str(exc)
        return state

    if not isinstance(raw, Mapping):
        return empty_vision_state("invalid")
    state = normalize_vision_state(raw, now=now)
    state["age_ms"] = int(age_s * 1000)
    return state


def _load_raw_state(path: Optional[os.PathLike[str] | str] = None) -> tuple[Dict[str, Any], Optional[str]]:
    state_path = Path(path) if path is not None else STATE_PATH
    try:
        with state_path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except FileNotFoundError:
        return {}, "missing"
    except Exception as exc:
        return {"error": str(exc)}, "read_error"
    return raw if isinstance(raw, dict) else {}, None


def normalize_gripper_state(payload: Mapping[str, Any], *, now: Optional[float] = None) -> Dict[str, Any]:
    now = time.time() if now is None else now
    angle = _number(payload.get("sg90_angle", payload.get("sg90_angle_deg", payload.get("angle_deg"))), 60.0) or 60.0
    state = str(payload.get("state") or ("CLOSED" if angle >= 120 else "OPEN")).upper()
    if state not in {"OPEN", "CLOSED", "MOVING"}:
        state = "CLOSED" if angle >= 120 else "OPEN"
    return {
        "state": state,
        "sg90_angle": angle,
        "sg90_angle_deg": angle,
        "tof_mm": _number(payload.get("tof_mm")),
        "source": payload.get("source", "runtime_state_file"),
        "fresh": True,
        "updated_at": _number(payload.get("updated_at", payload.get("timestamp", payload.get("ts"))), now) or now,
    }


def load_latest_runtime_state(path: Optional[os.PathLike[str] | str] = None,
                              ttl_s: float = DEFAULT_TTL_S) -> Dict[str, Any]:
    now = time.time()
    raw, reason = _load_raw_state(path)
    if reason:
        return {"vision": empty_vision_state(reason), "gripper": empty_gripper_state(reason)}

    vision_raw = raw.get("vision") if isinstance(raw.get("vision"), Mapping) else raw
    vision = normalize_vision_state(vision_raw, now=now)
    vision_age = now - float(vision.get("updated_at", 0) or 0)
    if vision_age > ttl_s:
        vision = empty_vision_state("stale")
        vision["age_ms"] = int(vision_age * 1000)
    else:
        vision["age_ms"] = int(max(0.0, vision_age) * 1000)

    gripper_raw = raw.get("gripper") if isinstance(raw.get("gripper"), Mapping) else {}
    gripper = normalize_gripper_state(gripper_raw, now=now) if gripper_raw else empty_gripper_state("missing")
    gripper_age = now - float(gripper.get("updated_at", 0) or 0)
    if gripper_age > ttl_s * 4:
        gripper["fresh"] = False
        gripper["reason"] = "stale"
    gripper["age_ms"] = int(max(0.0, gripper_age) * 1000) if gripper_raw else None
    return {"vision": vision, "gripper": gripper}


def write_vision_state(payload: Mapping[str, Any],
                       path: Optional[os.PathLike[str] | str] = None) -> Path:
    state_path = Path(path) if path is not None else STATE_PATH
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state = normalize_vision_state(payload)
    tmp_path = state_path.with_suffix(state_path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as handle:
        json.dump(state, handle, ensure_ascii=False, separators=(",", ":"))
    os.replace(tmp_path, state_path)
    return state_path


def write_runtime_state(*,
                        vision: Optional[Mapping[str, Any]] = None,
                        gripper: Optional[Mapping[str, Any]] = None,
                        path: Optional[os.PathLike[str] | str] = None) -> Path:
    state_path = Path(path) if path is not None else STATE_PATH
    raw, _reason = _load_raw_state(state_path)
    if not isinstance(raw, dict):
        raw = {}
    if vision is not None:
        raw["vision"] = normalize_vision_state(vision)
    if gripper is not None:
        raw["gripper"] = normalize_gripper_state(gripper)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = state_path.with_suffix(state_path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as handle:
        json.dump(raw, handle, ensure_ascii=False, separators=(",", ":"))
    os.replace(tmp_path, state_path)
    return state_path


def clear_vision_state(path: Optional[os.PathLike[str] | str] = None) -> Path:
    return write_vision_state(empty_vision_state("clear"), path=path)
