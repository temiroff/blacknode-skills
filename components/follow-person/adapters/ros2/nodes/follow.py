"""ROS 2 visual-follow nodes: single-shot joint servo toward a CV2 detection.

``ROS2NativeFollowDetectionJoint`` and ``ROS2FollowDetectionJoint`` are
self-contained one-shot servo steps (native rclpy or rosbridge). The
persistent variant, ``RobotFollow``, delegates its
run lifecycle to :mod:`blacknode.pkg.blacknode_skills.follow_person.follow_runtime`
so Stop All and status reporting can reach it without importing this
node-decorated module.
"""
from __future__ import annotations

import math
import urllib.request
import json
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Int, Text, node

from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.pkg.blacknode_skills.follow_person import follow_runtime

_CATEGORY = "Skills"


def _resolve_transport(ctx: dict) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    if requested in {"native", "rosbridge"}:
        return requested
    native_ok, _ = nr.available()
    return "native" if native_ok else "rosbridge"


def _transport_report(ctx: dict, resolved: str) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    suffix = " (auto-selected)" if requested == "auto" else ""
    return f"transport: {resolved}{suffix}"


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _to_radians(value: float, units: str) -> float:
    return math.radians(value) if units == "degrees" else value


def _from_radians(value: float, units: str) -> float:
    return math.degrees(value) if units == "degrees" else value


def _detection_center_x(detection: dict[str, Any]) -> float | None:
    center = detection.get("center")
    if isinstance(center, dict):
        value = _finite_float(center.get("x"))
        if value is not None:
            return value
    return _finite_float(detection.get("center_x"))


def _detection_width(detection: dict[str, Any], fallback: int) -> float:
    for key in ("frame_width", "image_width", "width"):
        value = _finite_float(detection.get(key))
        if value and value > 0:
            return value
    for key in ("frame", "image", "metadata"):
        nested = detection.get(key)
        if isinstance(nested, dict):
            for width_key in ("width", "frame_width", "image_width"):
                value = _finite_float(nested.get(width_key))
                if value and value > 0:
                    return value
    return max(1.0, float(fallback or 1))


def _read_detection_url(url: str, timeout: float) -> tuple[dict[str, Any], str]:
    if not url:
        return {}, ""
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "BlacknodeROS2FollowDetection/0.1"})
        with urllib.request.urlopen(req, timeout=max(0.2, timeout)) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {}, f"{type(exc).__name__}: {exc}"
    if not isinstance(payload, dict):
        return {}, "detection URL did not return a JSON object"
    detection = payload.get("detection") if isinstance(payload.get("detection"), dict) else payload
    if isinstance(detection, dict) and "found" not in detection and "found" in payload:
        detection = {**detection, "found": bool(payload.get("found"))}
    return dict(detection), ""


@node(
    name="ROS2NativeFollowDetectionJoint",
    category=_CATEGORY,
    hidden=True,
    description="Visual-servo one joint toward a CV2 detection center through native rclpy. No rosbridge required; safe by default.",
    inputs={
        "trigger": AnyPort,
        "detection": Dict,
        "detection_url": Text(default=""),
        "robot": Dict,
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "frame_width": Int(default=640),
        "target_x": Float(default=0.5),
        "deadband": Float(default=0.08),
        "gain": Float(default=35.0),
        "max_step": Float(default=8.0),
        "invert": Bool(default=False),
        "ramp_seconds": Float(default=0.35),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "moved": Bool,
        "joint": Text,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "error": Float,
        "command": Float,
        "report": Text,
    },
)
def ros2_native_follow_detection_joint(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    detection = ctx.get("detection") if isinstance(ctx.get("detection"), dict) else {}
    detection_url = str(ctx.get("detection_url") or "").strip()
    if detection_url:
        fetched_detection, detection_error = _read_detection_url(detection_url, timeout=1.0)
        if fetched_detection:
            detection = fetched_detection
        elif not detection:
            detection = {"found": False, "error": detection_error, "detection_url": detection_url}
    blocked = {
        "moved": False,
        "joint": joint,
        "before": {},
        "after": {},
        "target": {},
        "error": 0.0,
        "command": 0.0,
    }
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to the actuator/joint that should follow the cube."}
    if not detection:
        return {**blocked, "report": "native follow detection: no CV2 detection payload yet."}
    if detection.get("found") is False:
        if detection.get("error"):
            return {**blocked, "report": f"native follow detection: could not read {detection_url or 'detection'} ({detection['error']})."}
        return {**blocked, "report": "native follow detection: CV2 does not currently see the target."}

    center_x = _detection_center_x(detection)
    if center_x is None:
        return {**blocked, "report": "native follow detection: detection has no center.x value."}

    width = _detection_width(detection, int(ctx.get("frame_width") or 640))
    target_x_value = _finite_float(ctx.get("target_x"))
    target_x = max(0.0, min(1.0, 0.5 if target_x_value is None else target_x_value))
    normalized_x = max(0.0, min(1.0, center_x / width))
    error = target_x - normalized_x
    if bool(ctx.get("invert", False)):
        error = -error

    deadband_value = _finite_float(ctx.get("deadband"))
    deadband = max(0.0, min(0.5, 0.08 if deadband_value is None else deadband_value))
    if normalized_x < target_x - deadband:
        zone = "LEFT"
    elif normalized_x > target_x + deadband:
        zone = "RIGHT"
    else:
        zone = "CENTER"
    gain_value = _finite_float(ctx.get("gain"))
    gain = 35.0 if gain_value is None else gain_value
    command = error * gain
    max_step_value = _finite_float(ctx.get("max_step"))
    max_step = abs(8.0 if max_step_value is None else max_step_value)
    if max_step > 0:
        command = max(-max_step, min(max_step, command))

    if abs(error) <= deadband:
        return {
            **blocked,
            "error": error,
            "command": 0.0,
            "report": (
                f"native follow {joint}: target centered enough "
                f"(zone={zone}, x={center_x:.1f}/{width:.0f}, error={error:+.3f}, deadband={deadband:g}); "
                "no command streamed."
            ),
        }

    if not bool(ctx.get("armed", False)):
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": (
                f"BLOCKED: native ROS 2 visual follow preview only. Set armed=true to move {joint}. "
                f"Cube zone={zone}, x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
                f"command={command:+.2f} {units}."
            ),
        }

    ok, err = nr.available()
    if not ok:
        return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: {err}"}

    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    ramp_seconds_value = _finite_float(ctx.get("ramp_seconds"))
    hold_seconds_value = _finite_float(ctx.get("hold_seconds"))
    rate_hz_value = _finite_float(ctx.get("rate_hz"))
    timeout_value = _finite_float(ctx.get("timeout"))
    ramp_seconds = 0.35 if ramp_seconds_value is None else ramp_seconds_value
    hold_seconds = 0.2 if hold_seconds_value is None else hold_seconds_value
    rate_hz = 30.0 if rate_hz_value is None else rate_hz_value
    timeout = 10.0 if timeout_value is None else timeout_value

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = nr.read_config(config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: {exc}"}
        if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
            return {
                **blocked,
                "error": error,
                "command": command,
                "report": "BLOCKED: the robot driver reports it is read-only (commands_allowed=false).",
            }

    try:
        start_rad = nr.read_pose(state_topic, timeout)
    except Exception as exc:
        return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}",
        }

    command_rad = _to_radians(command, units)
    raw_target_rad = start_rad[joint] + command_rad
    limits = nr.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    names = list(start_rad.keys())
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value

    result = nr.stream_motion(
        command_topic, names, start_rad, target_rad,
        ramp_seconds=ramp_seconds, hold_seconds=hold_seconds, rate_hz=rate_hz, timeout=timeout,
    )
    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "error": error,
            "command": command,
            "report": f"native follow {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = nr.read_pose(state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"
    report = (
        f"native follow {joint}: cube x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
        f"command={command:+.2f} {units}, target={target[joint]:.2f}{clamp_note}; "
        f"streamed {result.get('sent', 0)} commands at {rate_hz:g} Hz"
    )
    return {
        "moved": moved,
        "joint": joint,
        "before": before,
        "after": after,
        "target": target,
        "error": error,
        "command": command,
        "report": report,
    }


@node(
    name="ROS2FollowDetectionJoint",
    category=_CATEGORY,
    description="Visual-servo one joint using native ROS 2 or rosbridge automatically. Safe by default: disarmed.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "detection": Dict,
        "detection_url": Text(default=""),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "frame_width": Int(default=640),
        "target_x": Float(default=0.5),
        "deadband": Float(default=0.08),
        "gain": Float(default=35.0),
        "max_step": Float(default=8.0),
        "invert": Bool(default=False),
        "ramp_seconds": Float(default=0.35),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "moved": Bool,
        "joint": Text,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "error": Float,
        "command": Float,
        "report": Text,
    },
)
def ros2_follow_detection_joint(ctx: dict) -> dict:
    transport = _resolve_transport(ctx)
    if transport == "native":
        result = ros2_native_follow_detection_joint(ctx)
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    detection = ctx.get("detection") if isinstance(ctx.get("detection"), dict) else {}
    detection_url = str(ctx.get("detection_url") or "").strip()
    if detection_url:
        fetched_detection, detection_error = _read_detection_url(detection_url, timeout=1.0)
        if fetched_detection:
            detection = fetched_detection
        elif not detection:
            detection = {"found": False, "error": detection_error, "detection_url": detection_url}
    blocked = {
        "moved": False,
        "joint": joint,
        "before": {},
        "after": {},
        "target": {},
        "error": 0.0,
        "command": 0.0,
    }
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to the actuator/joint that should follow the cube."}
    if not detection:
        return {**blocked, "report": "follow detection: no CV2 detection payload yet."}
    if detection.get("found") is False:
        if detection.get("error"):
            return {**blocked, "report": f"follow detection: could not read {detection_url or 'detection'} ({detection['error']})."}
        return {**blocked, "report": "follow detection: CV2 does not currently see the target."}

    center_x = _detection_center_x(detection)
    if center_x is None:
        return {**blocked, "report": "follow detection: detection has no center.x value."}

    width = _detection_width(detection, int(ctx.get("frame_width") or 640))
    target_x_value = _finite_float(ctx.get("target_x"))
    target_x = max(0.0, min(1.0, 0.5 if target_x_value is None else target_x_value))
    normalized_x = max(0.0, min(1.0, center_x / width))
    error = target_x - normalized_x
    if bool(ctx.get("invert", False)):
        error = -error

    deadband_value = _finite_float(ctx.get("deadband"))
    deadband = max(0.0, min(0.5, 0.08 if deadband_value is None else deadband_value))
    if normalized_x < target_x - deadband:
        zone = "LEFT"
    elif normalized_x > target_x + deadband:
        zone = "RIGHT"
    else:
        zone = "CENTER"
    gain_value = _finite_float(ctx.get("gain"))
    gain = 35.0 if gain_value is None else gain_value
    command = error * gain
    max_step_value = _finite_float(ctx.get("max_step"))
    max_step = abs(8.0 if max_step_value is None else max_step_value)
    if max_step > 0:
        command = max(-max_step, min(max_step, command))

    if abs(error) <= deadband:
        return {
            **blocked,
            "error": error,
            "command": 0.0,
            "report": (
                f"follow {joint}: target centered enough "
                f"(zone={zone}, x={center_x:.1f}/{width:.0f}, error={error:+.3f}, deadband={deadband:g}); "
                "no command streamed."
            ),
        }

    if not bool(ctx.get("armed", False)):
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": (
                f"BLOCKED: visual follow preview only. Set armed=true to move {joint}. "
                f"Cube zone={zone}, x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
                f"command={command:+.2f} {units}."
            ),
        }

    ok, err = rb.available()
    if not ok:
        return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: {err}"}

    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    ramp_seconds_value = _finite_float(ctx.get("ramp_seconds"))
    hold_seconds_value = _finite_float(ctx.get("hold_seconds"))
    rate_hz_value = _finite_float(ctx.get("rate_hz"))
    timeout_value = _finite_float(ctx.get("timeout"))
    ramp_seconds = 0.35 if ramp_seconds_value is None else ramp_seconds_value
    hold_seconds = 0.2 if hold_seconds_value is None else hold_seconds_value
    rate_hz = 30.0 if rate_hz_value is None else rate_hz_value
    timeout = 10.0 if timeout_value is None else timeout_value

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: {exc}"}
        if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
            return {
                **blocked,
                "error": error,
                "command": command,
                "report": "BLOCKED: the robot bridge reports it is read-only (commands_allowed=false). Relaunch it to accept commands.",
            }

    try:
        start_rad = rb.read_pose(host, port, state_topic, timeout)
    except Exception as exc:
        return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}",
        }

    command_rad = _to_radians(command, units)
    raw_target_rad = start_rad[joint] + command_rad
    limits = rb.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    names = list(start_rad.keys())
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value

    result = rb.stream_motion(
        host, port, command_topic, names, start_rad, target_rad,
        ramp_seconds=ramp_seconds, hold_seconds=hold_seconds, rate_hz=rate_hz, timeout=timeout,
    )
    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "error": error,
            "command": command,
            "report": f"follow {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = rb.read_pose(host, port, state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"
    report = (
        f"follow {joint}: cube zone={zone}, x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
        f"command={command:+.2f} {units}, target={target[joint]:.2f}{clamp_note}; "
        f"streamed {result.get('sent', 0)} commands at {rate_hz:g} Hz"
    )
    return {
        "moved": moved,
        "joint": joint,
        "before": before,
        "after": after,
        "target": target,
        "error": error,
        "command": command,
        "report": report,
    }


@node(
    name="RobotFollow",
    live=True,
    category=_CATEGORY,
    description="Start one persistent visual-servo service with long-lived detection, joint-state, and command streams.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop", "check"], default="start"),
        "run_id": Text(default="vision_follow"),
        "loop_hz": Float(default=2.0),
        "detection": Dict,
        "detection_stream": Dict,
        "detection_url": Text(default=""),
        "detection_timeout": Float(default=1.0),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "frame_width": Int(default=640),
        "target_x": Float(default=0.5),
        "deadband": Float(default=0.08),
        "gain": Float(default=35.0),
        "max_step": Float(default=8.0),
        "invert": Bool(default=False),
        "ramp_seconds": Float(default=0.35),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool,
        "moved": Bool,
        "joint": Text,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "error": Float,
        "command": Float,
        "report": Text,
    },
)
def ros2_continuous_follow_detection_joint(ctx: dict) -> dict:
    return follow_runtime.run_continuous_follow(ctx)
