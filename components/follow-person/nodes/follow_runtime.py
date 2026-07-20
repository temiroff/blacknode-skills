"""Background visual-follow service shared by the ROS 2 follow-person adapter.

Owns the persistent visual-servo loop (long-lived detection, joint-state, and
command streams) started by ``ROS2ContinuousFollowDetectionJoint``. The node
module stays a thin dispatcher; this module owns run state, the worker
thread, and lifecycle (start/stop/status) so it can be reached from Stop All
without importing node-decorated code.
"""
from __future__ import annotations

import json
import math
import threading
import time
import urllib.request
from typing import Any

from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb

_continuous_follow_lock = threading.Lock()
_continuous_follow_runs: dict[str, dict[str, Any]] = {}


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


def _continuous_follow_result(
    *, joint: str = "", report: str = "", running: bool = False,
) -> dict[str, Any]:
    return {
        "running": running,
        "moved": False,
        "joint": joint,
        "before": {},
        "after": {},
        "target": {},
        "error": 0.0,
        "command": 0.0,
        "report": report,
    }


def _stop_continuous_follow(run_id: str) -> bool:
    with _continuous_follow_lock:
        item = _continuous_follow_runs.pop(run_id, None)
    if item is None:
        return False
    item["stop"].set()
    thread = item.get("thread")
    if thread is not None and thread is not threading.current_thread():
        thread.join(timeout=2.0)
    return True


def stop_continuous_follow_services() -> dict[str, Any]:
    """Stop every background visual-follow loop owned by this process."""
    with _continuous_follow_lock:
        run_ids = list(_continuous_follow_runs)
    stopped = sum(1 for run_id in run_ids if _stop_continuous_follow(run_id))
    return {
        "ok": True,
        "stopped": stopped,
        "report": f"stopped {stopped} continuous visual-follow loop(s)",
    }


def continuous_follow_runtime_status() -> list[dict[str, Any]]:
    with _continuous_follow_lock:
        return [
            {
                "run_id": run_id,
                "joint": str(item.get("ctx", {}).get("joint") or ""),
                "loop_hz": float(item.get("ctx", {}).get("loop_hz") or 2.0),
                "detection_stream": str(item.get("ctx", {}).get("detection_stream", {}).get("stream_id") or ""),
                "session_resets": int(item.get("session_resets") or 0),
                "report": str(item.get("last", {}).get("report") or ""),
            }
            for run_id, item in _continuous_follow_runs.items()
            if item.get("thread") is not None and item["thread"].is_alive()
        ]


def _continuous_detection_url(ctx: dict[str, Any]) -> str:
    stream = ctx.get("detection_stream")
    if isinstance(stream, dict):
        url = str(stream.get("url") or stream.get("detection_url") or "").strip()
        if url:
            return url
    return str(ctx.get("detection_url") or "").strip()


def _continuous_follow_step(item: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    """Run one lightweight tick using persistent ROS subscription entities."""
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    detection_url = _continuous_detection_url(ctx)
    detection, detection_error = _read_detection_url(detection_url, timeout=0.5)
    if not detection:
        return _continuous_follow_result(joint=joint, report=f"follow detection stream unavailable: {detection_error}")
    updated_at = _finite_float(detection.get("updated_at"))
    detection_timeout_value = _finite_float(ctx.get("detection_timeout"))
    detection_timeout = max(0.1, 1.0 if detection_timeout_value is None else detection_timeout_value)
    if updated_at and time.time() - updated_at > detection_timeout:
        return _continuous_follow_result(
            joint=joint,
            report=f"follow detection stream stale ({time.time() - updated_at:.2f}s > {detection_timeout:g}s); command suppressed.",
        )
    payload = detection.get("detection") if isinstance(detection.get("detection"), dict) else detection
    if payload.get("found") is False or detection.get("found") is False:
        return _continuous_follow_result(joint=joint, report="follow detection: target not visible; command suppressed.")
    center_x = _detection_center_x(payload)
    if center_x is None:
        return _continuous_follow_result(joint=joint, report="follow detection: stream has no center.x value.")

    width = _detection_width(payload, int(ctx.get("frame_width") or 640))
    target_x_value = _finite_float(ctx.get("target_x"))
    target_x = max(0.0, min(1.0, 0.5 if target_x_value is None else target_x_value))
    normalized_x = max(0.0, min(1.0, center_x / width))
    error = target_x - normalized_x
    if bool(ctx.get("invert", False)):
        error = -error
    deadband_value = _finite_float(ctx.get("deadband"))
    deadband = max(0.0, min(0.5, 0.08 if deadband_value is None else deadband_value))
    if abs(error) <= deadband:
        return {
            **_continuous_follow_result(joint=joint),
            "error": error,
            "report": f"follow {joint}: target centered; persistent stream active, no command.",
        }

    gain_value = _finite_float(ctx.get("gain"))
    command = error * (35.0 if gain_value is None else gain_value)
    max_step_value = _finite_float(ctx.get("max_step"))
    max_step = abs(8.0 if max_step_value is None else max_step_value)
    if max_step > 0:
        command = max(-max_step, min(max_step, command))

    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    signature = (host, port, state_topic, command_topic, config_topic)
    if item.get("session_signature") != signature:
        previous_session = item.get("session")
        if previous_session is not None:
            rb.release_joint_stream(previous_session, discard=True)
        item["session"] = rb.acquire_joint_stream(*signature, timeout=min(2.0, float(ctx.get("timeout") or 10.0)))
        item["session_signature"] = signature
    session = item["session"]
    pose, config, state_age = session.snapshot()
    if not pose:
        pose = session.wait_for_pose(1.0)
        pose, config, state_age = session.snapshot()
    state_timeout = max(0.25, 3.0 / max(0.2, float(ctx.get("loop_hz") or 2.0)))
    if not pose or state_age > state_timeout:
        # A roslibpy Topic can remain registered but stop dispatching after a
        # robot driver disappears and comes back. Replace that stale session;
        # the next controller tick acquires fresh subscription entities.
        rb.release_joint_stream(item.get("session"), discard=True)
        item["session"] = None
        item["session_signature"] = None
        item["command_target"] = None
        item["target_joint"] = None
        item["session_resets"] = int(item.get("session_resets") or 0) + 1
        return _continuous_follow_result(
            joint=joint,
            report=f"follow {joint}: joint-state stream missing or stale; resetting subscription, command suppressed.",
        )
    if config_topic and not config:
        config = session.wait_for_config(1.0)
        if not config:
            return _continuous_follow_result(
                joint=joint,
                report=f"follow {joint}: waiting for safety config on {config_topic}; command suppressed.",
            )
    if joint not in pose:
        return _continuous_follow_result(
            joint=joint,
            report=f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(pose)}",
        )
    if config and config.get("commands_allowed") is False:
        return _continuous_follow_result(joint=joint, report="BLOCKED: robot bridge is read-only (commands_allowed=false).")

    pose_value = float(pose[joint])
    previous_target = _finite_float(item.get("command_target"))
    max_lead = max(math.radians(2.0), abs(_to_radians(max_step * 4.0, units)))
    if (
        item.get("target_joint") != joint
        or previous_target is None
        or abs(previous_target - pose_value) > max_lead
    ):
        previous_target = pose_value
    raw_target = previous_target + _to_radians(command, units)
    # Accumulate sub-degree corrections so they overcome servo friction, but
    # never let the desired setpoint run away from live hardware feedback.
    raw_target = min(pose_value + max_lead, max(pose_value - max_lead, raw_target))
    target_value = raw_target
    limits = rb.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_value = min(upper, max(lower, raw_target))
    item["command_target"] = target_value
    item["target_joint"] = joint
    target_rad = dict(pose)
    target_rad[joint] = target_value
    session.publish(target_rad)
    before = {name: _from_radians(value, units) for name, value in pose.items()}
    target = {name: _from_radians(value, units) for name, value in target_rad.items()}
    clamp_note = "" if abs(raw_target - target_value) < 1e-9 else " (clamped)"
    return {
        "running": True,
        "moved": abs(target_value - pose[joint]) > 1e-9,
        "joint": joint,
        "before": before,
        "after": before,
        "target": target,
        "error": error,
        "command": command,
        "report": (
            f"follow {joint}: persistent streams, x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
            f"command={command:+.2f} {units}, target={target[joint]:.2f}{clamp_note}"
        ),
    }


def _continuous_follow_worker(run_id: str, item: dict[str, Any]) -> None:
    stop_event = item["stop"]
    try:
        while not stop_event.is_set():
            with _continuous_follow_lock:
                current = _continuous_follow_runs.get(run_id)
                if current is not item:
                    return
                step_ctx = dict(item["ctx"])
            step_ctx["armed"] = True
            try:
                result = _continuous_follow_step(item, step_ctx)
            except Exception as exc:
                rb.release_joint_stream(item.get("session"), discard=True)
                item["session"] = None
                item["session_signature"] = None
                result = _continuous_follow_result(
                    joint=str(step_ctx.get("joint") or ""),
                    report=f"continuous follow FAILED: {exc}; reconnecting.",
                )
            with _continuous_follow_lock:
                if _continuous_follow_runs.get(run_id) is item:
                    item["last"] = result
            loop_hz_value = _finite_float(step_ctx.get("loop_hz"))
            loop_hz = max(0.2, min(20.0, 2.0 if loop_hz_value is None else loop_hz_value))
            stop_event.wait(1.0 / loop_hz)
    finally:
        rb.release_joint_stream(item.get("session"))
        item["session"] = None
        item["session_signature"] = None


def run_continuous_follow(ctx: dict) -> dict:
    """Handle the start/stop/check action dispatch for one persistent follow run."""
    action = str(ctx.get("action") or "start").strip().lower()
    run_id = str(ctx.get("run_id") or "vision_follow").strip() or "vision_follow"
    joint = str(ctx.get("joint") or "").strip()

    if action == "stop":
        stopped = _stop_continuous_follow(run_id)
        return _continuous_follow_result(
            joint=joint,
            report=f"continuous follow '{run_id}' {'stopped' if stopped else 'was not running'}.",
        )

    with _continuous_follow_lock:
        existing = _continuous_follow_runs.get(run_id)
        if existing is not None and not existing["thread"].is_alive():
            _continuous_follow_runs.pop(run_id, None)
            existing = None

    if action == "check":
        if existing is None:
            return _continuous_follow_result(joint=joint, report=f"continuous follow '{run_id}' is not running.")
        with _continuous_follow_lock:
            latest = dict(existing.get("last") or _continuous_follow_result(joint=joint))
        latest["running"] = True
        latest["report"] = f"continuous follow '{run_id}' is running; {latest.get('report') or 'waiting for first update'}"
        return latest

    if not bool(ctx.get("armed", False)):
        _stop_continuous_follow(run_id)
        return _continuous_follow_result(
            joint=joint,
            report="BLOCKED: set armed=true, then cook once to start continuous movement.",
        )
    if not _continuous_detection_url(ctx):
        _stop_continuous_follow(run_id)
        return _continuous_follow_result(
            joint=joint,
            report="BLOCKED: connect a CV2 detection_stream (or legacy detection_url), then cook once.",
        )

    if existing is not None:
        with _continuous_follow_lock:
            existing["ctx"] = dict(ctx)
            latest = dict(existing.get("last") or _continuous_follow_result(joint=joint))
        latest["running"] = True
        latest["report"] = f"continuous follow '{run_id}' updated and remains running; {latest.get('report') or 'waiting for first update'}"
        return latest

    item: dict[str, Any] = {
        "ctx": dict(ctx),
        "stop": threading.Event(),
        "session": None,
        "session_signature": None,
        "command_target": None,
        "target_joint": None,
        "session_resets": 0,
        "last": _continuous_follow_result(joint=joint, report="waiting for first update"),
    }
    thread = threading.Thread(
        target=_continuous_follow_worker,
        args=(run_id, item),
        name=f"blacknode-follow-{run_id}",
        daemon=True,
    )
    item["thread"] = thread
    with _continuous_follow_lock:
        _continuous_follow_runs[run_id] = item
    thread.start()
    loop_hz_value = _finite_float(ctx.get("loop_hz"))
    loop_hz = 2.0 if loop_hz_value is None else loop_hz_value
    return _continuous_follow_result(
        joint=joint,
        running=True,
        report=f"persistent follow '{run_id}' started at {loop_hz:g} Hz; cook is complete, use action=stop or Stop All to stop.",
    )
