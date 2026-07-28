"""Managed ROS 2 leader joint-state subscriptions for split follow graphs."""
from __future__ import annotations

import math
import threading
import time
from typing import Any

from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.pkg.blacknode_ros2 import sample_stream

_lock = threading.RLock()
_runs: dict[str, dict[str, Any]] = {}


def _transport(ctx: dict[str, Any]) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    if requested in {"native", "rosbridge"}:
        return requested
    available, _ = nr.available()
    return "native" if available else "rosbridge"


def _sample(item: dict[str, Any]) -> dict[str, Any]:
    pose, config, age = item["session"].snapshot()
    return {
        "kind": "blacknode.ros2-joint-subscription",
        "schema_version": 1,
        "run_id": item["run_id"],
        "state_topic": item["state_topic"],
        "config_topic": item["config_topic"],
        "transport": item["transport"],
        "pose": dict(pose or {}),
        "config": dict(config or {}),
        "age": float(age),
        "hardware_id": item["hardware_id"],
        "calibration_path": item["calibration_path"],
        "captured_at_ns": time.time_ns(),
    }


def subscription_snapshot(subscription: dict[str, Any]) -> dict[str, Any]:
    run_id = str(subscription.get("run_id") or "").strip()
    with _lock:
        item = _runs.get(run_id)
        if item is None:
            return {}
        return _sample(item)


def _stop(run_id: str) -> bool:
    with _lock:
        item = _runs.pop(run_id, None)
    if item is None:
        return False
    runtime = nr if item["transport"] == "native" else rb
    runtime.release_joint_stream(item["session"], discard=True)
    sample_stream.unregister(run_id)
    return True


def stop_leader_subscription_services() -> dict[str, Any]:
    with _lock:
        run_ids = list(_runs)
    stopped = sum(1 for run_id in run_ids if _stop(run_id))
    return {
        "ok": True,
        "stopped": stopped,
        "report": f"stopped {stopped} leader subscription(s)",
    }


def run_leader_subscription(ctx: dict[str, Any]) -> dict[str, Any]:
    action = str(ctx.get("action") or "start").strip().lower()
    run_id = str(ctx.get("run_id") or "leader_joint_subscription").strip()
    run_id = run_id or "leader_joint_subscription"
    if action == "stop":
        stopped = _stop(run_id)
        return {
            "running": False,
            "live": False,
            "subscription": {},
            "sample_stream": {},
            "pose": {},
            "age": -1.0,
            "report": f"leader subscription '{run_id}' {'stopped' if stopped else 'was not running'}.",
        }

    with _lock:
        existing = _runs.get(run_id)
    if action == "check" and existing is None:
        return {
            "running": False, "live": False, "subscription": {},
            "sample_stream": {}, "pose": {}, "age": -1.0,
            "report": f"leader subscription '{run_id}' is not running.",
        }

    robot = ctx.get("leader_robot") if isinstance(ctx.get("leader_robot"), dict) else {}
    driver = robot.get("driver") if isinstance(robot.get("driver"), dict) else {}
    transport = _transport(ctx)
    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/leader/joint_states")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "/leader/joint_config")
    signature = (
        (state_topic, config_topic)
        if transport == "native"
        else (host, port, state_topic, config_topic)
    )
    if existing is not None and existing["signature"] != signature:
        _stop(run_id)
        existing = None
    if existing is None:
        timeout = min(2.0, float(ctx.get("timeout") or 10.0))
        if transport == "native":
            session = nr.acquire_joint_stream(
                state_topic, "", config_topic, timeout=timeout,
                node_name=f"blacknode_{run_id}_subscriber",
            )
        else:
            session = rb.acquire_joint_stream(
                host, port, state_topic, "", config_topic, timeout=timeout,
            )
        item = {
            "run_id": run_id,
            "session": session,
            "signature": signature,
            "transport": transport,
            "state_topic": state_topic,
            "config_topic": config_topic,
            "hardware_id": str(driver.get("hardware_id") or ""),
            "calibration_path": str(driver.get("calibration_path") or ""),
        }
        item["sample_stream"] = sample_stream.register(run_id, lambda: _sample(item))
        with _lock:
            _runs[run_id] = item
        existing = item

    snapshot = _sample(existing)
    stale_after = max(0.25, float(ctx.get("stale_after") or 0.75))
    live = bool(snapshot["pose"]) and snapshot["age"] <= stale_after
    descriptor = {
        "kind": "blacknode.ros2-joint-subscription",
        "schema_version": 1,
        "run_id": run_id,
        "state_topic": state_topic,
        "config_topic": config_topic,
        "transport": transport,
        "sample_stream": dict(existing["sample_stream"]),
    }
    pose_degrees = {
        name: math.degrees(float(value))
        for name, value in snapshot["pose"].items()
    }
    return {
        "running": True,
        "live": live,
        "subscription": descriptor,
        "sample_stream": dict(existing["sample_stream"]),
        "pose": pose_degrees,
        "age": snapshot["age"],
        "report": (
            f"subscribed to {state_topic} via {transport}"
            if live
            else f"waiting for fresh {state_topic} messages via {transport}"
        ),
    }
