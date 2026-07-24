"""Background leader/follower teleoperation service shared by the ROS 2 follow-person adapter.

Owns the persistent leader-pose-to-follower-pose streaming loop started by
``ROS2LeaderFollower``. The node module stays a thin dispatcher; this module
owns run state, the worker thread, and lifecycle (start/stop/status) so it
can be reached from Stop All without importing node-decorated code.
"""
from __future__ import annotations

import base64
import html
import math
import threading
import time
from typing import Any

from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.pkg.blacknode_ros2 import sample_stream

_leader_follower_lock = threading.Lock()
_leader_follower_runs: dict[str, dict[str, Any]] = {}


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _resolve_transport(ctx: dict) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    if requested in {"native", "rosbridge"}:
        return requested
    native_ok, _ = nr.available()
    return "native" if native_ok else "rosbridge"


def _driver_is_calibrated(driver: dict[str, Any]) -> bool:
    profile = driver.get("profile") if isinstance(driver.get("profile"), dict) else {}
    calibration = (
        profile.get("calibration")
        if isinstance(profile.get("calibration"), dict)
        else {}
    )
    return bool(driver.get("calibration_path") or calibration)


def _endpoint(
    ctx: dict[str, Any],
    role: str,
    fallback_host: str,
    fallback_port: int,
) -> tuple[str, int]:
    host = str(ctx.get(f"{role}_host") or fallback_host)
    requested_port = int(ctx.get(f"{role}_port") or 0)
    return host, requested_port or fallback_port


def _svg_text(value: Any, limit: int = 90) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        text = text[: limit - 3] + "..."
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


def _leader_follower_result(
    *, running: bool = False, armed: bool = False, live: bool = False,
    commanded: bool = False, leader_pose: dict[str, float] | None = None,
    follower_pose: dict[str, float] | None = None, target: dict[str, float] | None = None,
    clamped: list[str] | None = None, report: str = "",
) -> dict[str, Any]:
    leader_pose = dict(leader_pose or {})
    follower_pose = dict(follower_pose or {})
    target = dict(target or {})
    clamped = list(clamped or [])
    state = "ARMED / FOLLOWING" if armed and commanded else "ARMED / WAITING" if armed else "PREVIEW / DISARMED"
    accent = "#22c55e" if armed and live else "#f59e0b"
    names = list(target or leader_pose)
    display_names = names[:6]
    rows = []
    for index, name in enumerate(display_names):
        y = 246 + index * 42
        leader_value = leader_pose.get(name)
        follower_value = follower_pose.get(name)
        target_value = target.get(name)
        rows.append(
            f'<text x="48" y="{y}" fill="#cbd5e1" font-family="monospace" font-size="14">{_svg_text(name, 20)}</text>'
            f'<text x="360" y="{y}" text-anchor="end" fill="#f8fafc" font-family="monospace" font-size="14">{leader_value:.2f}°</text>'
            f'<text x="570" y="{y}" text-anchor="end" fill="#f8fafc" font-family="monospace" font-size="14">{target_value:.2f}°</text>'
            f'<text x="780" y="{y}" text-anchor="end" fill="#f8fafc" font-family="monospace" font-size="14">{follower_value:.2f}°</text>'
        )
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="840" height="560" viewBox="0 0 840 560">
<rect width="840" height="560" rx="24" fill="#0b1020"/>
<rect x="24" y="24" width="792" height="112" rx="18" fill="#172033" stroke="{accent}" stroke-width="2"/>
<text x="48" y="66" fill="#f8fafc" font-family="Arial,sans-serif" font-size="24" font-weight="800">LEADER → FOLLOWER</text>
<text x="48" y="98" fill="#93a4b8" font-family="Arial,sans-serif" font-size="14">Move the released leader by hand; follower commands remain bounded and clamped.</text>
<rect x="590" y="50" width="198" height="48" rx="24" fill="{accent}"/>
<text x="689" y="80" text-anchor="middle" fill="#07111f" font-family="Arial,sans-serif" font-size="14" font-weight="900">{state}</text>
<rect x="24" y="158" width="792" height="330" rx="18" fill="#172033"/>
<text x="48" y="198" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">JOINT</text>
<text x="360" y="198" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">LEADER</text>
<text x="570" y="198" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">SAFE TARGET</text>
<text x="780" y="198" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">FOLLOWER</text>
<line x1="48" y1="212" x2="780" y2="212" stroke="#334155"/>
{''.join(rows) if rows else '<text x="48" y="252" fill="#f59e0b" font-family="Arial,sans-serif" font-size="16">Waiting for both robot joint streams…</text>'}
{f'<text x="48" y="478" fill="#64748b" font-family="Arial,sans-serif" font-size="12">+{len(names) - 6} additional mapped joint(s) in summary output</text>' if len(names) > 6 else ''}
<text x="48" y="522" fill="{accent}" font-family="Arial,sans-serif" font-size="14" font-weight="700">{_svg_text(report, 105)}</text>
</svg>'''
    summary = {
        "running": running, "live": live, "armed": armed, "commanded": commanded,
        "leader_pose": leader_pose, "follower_pose": follower_pose, "target": target,
        "joints": names, "clamped": clamped, "report": report,
    }
    return {
        **summary,
        "joint_count": len(names),
        "dashboard": _svg_data(svg),
        "summary": summary,
    }


def _stop_leader_follower(run_id: str) -> bool:
    with _leader_follower_lock:
        item = _leader_follower_runs.pop(run_id, None)
    if item is None:
        return False
    sample_stream.unregister(run_id)
    item["stop"].set()
    thread = item.get("thread")
    if thread is not None and thread is not threading.current_thread():
        thread.join(timeout=2.0)
    return True


def stop_leader_follower_services() -> dict[str, Any]:
    with _leader_follower_lock:
        run_ids = list(_leader_follower_runs)
    stopped = sum(1 for run_id in run_ids if _stop_leader_follower(run_id))
    return {"ok": True, "stopped": stopped, "report": f"stopped {stopped} leader-follower controller(s)"}


def update_leader_follower_config(run_id: str, values: dict[str, Any]) -> dict[str, Any]:
    with _leader_follower_lock:
        item = _leader_follower_runs.get(run_id)
        if item is None:
            return {"ok": False, "report": f"leader-follower '{run_id}' is not running"}
        item["ctx"].update(values)
    return {"ok": True, "report": f"updated leader-follower '{run_id}'"}


def leader_follower_runtime_status() -> list[dict[str, Any]]:
    with _leader_follower_lock:
        return [
            {
                "run_id": run_id,
                "armed": bool(item.get("ctx", {}).get("armed", False)),
                "loop_hz": float(item.get("ctx", {}).get("loop_hz") or 60.0),
                "sample_stream": dict(item.get("sample_stream") or {}),
                "report": str(item.get("last", {}).get("report") or ""),
            }
            for run_id, item in _leader_follower_runs.items()
            if item.get("thread") is not None and item["thread"].is_alive()
        ]


def monitor_entries() -> list[dict[str, Any]]:
    """Entries for the live-node-monitor panel: one per active leader-follower run."""
    with _leader_follower_lock:
        return [
            {
                "run_id": run_id,
                "node_id": str(item.get("ctx", {}).get("__node_id__") or ""),
                "node_type": "ROS2LeaderFollower",
                "outputs": dict(item.get("last") or {}),
                "updated_at": item.get("updated_at"),
                "error": "",
            }
            for run_id, item in _leader_follower_runs.items()
        ]


def _leader_follower_step(item: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    armed = bool(ctx.get("armed", False))
    leader = ctx.get("leader_robot") if isinstance(ctx.get("leader_robot"), dict) else {}
    follower = ctx.get("follower_robot") if isinstance(ctx.get("follower_robot"), dict) else {}
    leader_driver = leader.get("driver") if isinstance(leader.get("driver"), dict) else {}
    follower_driver = follower.get("driver") if isinstance(follower.get("driver"), dict) else {}
    leader_id = str(leader_driver.get("hardware_id") or "")
    follower_id = str(follower_driver.get("hardware_id") or "")
    remote_leader = not bool(leader)
    if not follower_id:
        return _leader_follower_result(
            running=True,
            armed=armed,
            report="BLOCKED: the follower needs a USB hardware identity.",
        )
    if not remote_leader and not leader_id:
        return _leader_follower_result(
            running=True,
            armed=armed,
            report="BLOCKED: the leader needs a USB hardware identity.",
        )
    if leader_id and follower_id and leader_id == follower_id:
        return _leader_follower_result(running=True, armed=armed, report="BLOCKED: leader and follower resolve to the same USB device.")
    if bool(ctx.get("require_calibration", True)) and (
        (not remote_leader and not _driver_is_calibrated(leader_driver))
        or not _driver_is_calibrated(follower_driver)
    ):
        return _leader_follower_result(running=True, armed=armed, report="BLOCKED: save hardware calibration for both leader and follower.")
    if _resolve_transport(ctx) != "rosbridge":
        return _leader_follower_result(running=True, armed=armed, report="BLOCKED: leader-follower currently requires rosbridge transport.")

    host = str(ctx.get("host") or leader.get("host") or follower.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or leader.get("port") or follower.get("port") or 9090)
    leader_host, leader_port = _endpoint(ctx, "leader", host, port)
    follower_host, follower_port = _endpoint(ctx, "follower", host, port)
    placement = str(ctx.get("placement") or "same_device").strip().lower()
    if (
        placement == "separate_devices"
        and leader_host.strip().lower() in {"127.0.0.1", "localhost", "::1"}
    ):
        return _leader_follower_result(
            running=True,
            armed=armed,
            report=(
                "BLOCKED: separate_devices needs leader_host set to the leader "
                "computer's LAN IP or hostname."
            ),
        )
    leader_signature = (
        leader_host,
        leader_port,
        str(leader.get("state_topic") or "/leader/joint_states"),
        str(leader.get("command_topic") or "/leader/joint_commands"),
        str(leader.get("config_topic") or "/leader/joint_config"),
    )
    follower_signature = (
        follower_host,
        follower_port,
        str(follower.get("state_topic") or "/follower/joint_states"),
        str(follower.get("command_topic") or "/follower/joint_commands"),
        str(follower.get("config_topic") or "/follower/joint_config"),
    )
    for role, signature in (("leader", leader_signature), ("follower", follower_signature)):
        if item.get(f"{role}_signature") != signature:
            rb.release_joint_stream(item.get(f"{role}_session"), discard=True)
            item[f"{role}_session"] = rb.acquire_joint_stream(*signature, timeout=min(2.0, float(ctx.get("timeout") or 10.0)))
            item[f"{role}_signature"] = signature
    leader_session = item["leader_session"]
    follower_session = item["follower_session"]
    leader_pose_rad, leader_config, leader_age = leader_session.snapshot()
    follower_pose_rad, follower_config, follower_age = follower_session.snapshot()
    if not leader_pose_rad:
        leader_session.wait_for_pose(1.0)
        leader_pose_rad, leader_config, leader_age = leader_session.snapshot()
    if not follower_pose_rad:
        follower_session.wait_for_pose(1.0)
        follower_pose_rad, follower_config, follower_age = follower_session.snapshot()
    stale_after = max(0.25, float(ctx.get("stale_after") or 0.75))
    stream_issues: list[str] = []
    for role, session, pose, age in (
        ("leader", leader_session, leader_pose_rad, leader_age),
        ("follower", follower_session, follower_pose_rad, follower_age),
    ):
        if pose and age <= stale_after:
            continue
        issue = "missing" if not pose else f"stale ({age:.2f}s > {stale_after:.2f}s)"
        stream_issues.append(f"{role} {issue}")
        rb.release_joint_stream(session, discard=True)
        item[f"{role}_session"] = None
        item[f"{role}_signature"] = None
        item[f"{role}_session_resets"] = int(item.get(f"{role}_session_resets") or 0) + 1
    if stream_issues:
        return _leader_follower_result(
            running=True,
            armed=armed,
            report=(
                f"WAITING: {', '.join(stream_issues)} joint stream; resetting subscription; "
                "commands suppressed."
            ),
        )
    if not leader_config:
        leader_config = leader_session.wait_for_config(0.5)
    if not follower_config:
        follower_config = follower_session.wait_for_config(0.5)

    units = "degrees"
    leader_pose = {name: math.degrees(value) for name, value in leader_pose_rad.items()}
    follower_pose = {name: math.degrees(value) for name, value in follower_pose_rad.items()}
    joint_map = ctx.get("joint_map") if isinstance(ctx.get("joint_map"), dict) else {}
    mapping = {str(src): str(dst) for src, dst in joint_map.items()} if joint_map else {
        name: name for name in leader_pose_rad if name in follower_pose_rad
    }
    scales = ctx.get("scale") if isinstance(ctx.get("scale"), dict) else {}
    offsets = ctx.get("offset_deg") if isinstance(ctx.get("offset_deg"), dict) else {}
    limits = rb.limits_radians(follower_config)
    target_rad = dict(follower_pose_rad)
    target: dict[str, float] = {}
    leader_display: dict[str, float] = {}
    follower_display: dict[str, float] = {}
    clamped: list[str] = []
    tracking_mode = str(ctx.get("tracking_mode") or "direct").strip().lower()
    if tracking_mode not in {"bounded", "direct"}:
        tracking_mode = "bounded"
    max_step_value = _finite_float(ctx.get("max_step_deg"))
    deadband_value = _finite_float(ctx.get("deadband_deg"))
    max_step = math.radians(max(0.05, 2.0 if max_step_value is None else max_step_value))
    deadband = math.radians(max(0.0, 0.0 if deadband_value is None else deadband_value))
    for source, destination in mapping.items():
        if source not in leader_pose_rad or destination not in follower_pose_rad:
            continue
        scale = _finite_float(scales.get(source))
        offset = _finite_float(offsets.get(source))
        desired = leader_pose_rad[source] * (1.0 if scale is None else scale) + math.radians(0.0 if offset is None else offset)
        bounded = desired if tracking_mode == "direct" else min(
            follower_pose_rad[destination] + max_step,
            max(follower_pose_rad[destination] - max_step, desired),
        )
        if destination in limits:
            lower, upper = limits[destination]
            limited = min(upper, max(lower, bounded))
            if abs(limited - desired) > 1e-9:
                clamped.append(destination)
            bounded = limited
        target_rad[destination] = bounded
        leader_display[destination] = math.degrees(leader_pose_rad[source])
        follower_display[destination] = math.degrees(follower_pose_rad[destination])
        target[destination] = math.degrees(bounded)
    if not target:
        return _leader_follower_result(running=True, armed=armed, live=True, report="BLOCKED: leader and follower have no mapped joint names.")
    if bool(ctx.get("require_leader_released", True)) and leader_config.get("torque_enabled") is not False:
        return _leader_follower_result(running=True, armed=armed, live=True, leader_pose=leader_display, follower_pose=follower_display, target=target, report="BLOCKED: release leader torque before following.")
    if armed and (not follower_config or follower_config.get("commands_allowed") is False or any(name not in limits for name in target)):
        return _leader_follower_result(running=True, armed=armed, live=True, leader_pose=leader_display, follower_pose=follower_display, target=target, report="BLOCKED: follower safety config/limits are incomplete.")
    commanded = armed and any(abs(target_rad[name] - follower_pose_rad[name]) > deadband for name in target)
    if commanded:
        follower_session.publish(target_rad)
    sequence = int(item.get("sample_sequence") or 0) + 1
    captured_at_ns = time.time_ns()
    leader_sample = {
        destination: float(leader_pose_rad[source])
        for source, destination in mapping.items()
        if destination in target and source in leader_pose_rad
    }
    item["sample_sequence"] = sequence
    item["sample"] = {
        "kind": "blacknode.teleoperation-sample",
        "schema_version": 1,
        "sequence": sequence,
        "captured_at_ns": captured_at_ns,
        "monotonic_ns": time.monotonic_ns(),
        "leader_source_at_ns": captured_at_ns - int(max(0.0, leader_age) * 1_000_000_000),
        "observation_source_at_ns": captured_at_ns - int(max(0.0, follower_age) * 1_000_000_000),
        "joint_names": list(target),
        "leader": leader_sample,
        "observation": {name: float(follower_pose_rad[name]) for name in target},
        "action": {name: float(target_rad[name]) for name in target},
        "units": "radians",
        "armed": armed,
        "live": True,
        "commanded": commanded,
        "clamped": list(clamped),
        "leader_hardware_id": leader_id or f"remote:{leader_host}:{leader_port}",
        "follower_hardware_id": follower_id,
        "leader_calibration_path": str(leader_driver.get("calibration_path") or ""),
        "follower_calibration_path": str(follower_driver.get("calibration_path") or ""),
    }
    report = (
        f"Direct-following {len(target)} joint(s) from the live leader pose." if commanded and tracking_mode == "direct"
        else f"Following {len(target)} joint(s) at bounded targets." if commanded
        else f"Previewing {len(target)} mapped joint(s); set armed=true to move follower." if not armed
        else "Leader and follower are within the deadband."
    )
    return _leader_follower_result(
        running=True, armed=armed, live=True, commanded=commanded,
        leader_pose=leader_display, follower_pose=follower_display, target=target,
        clamped=clamped, report=report,
    )


def _leader_follower_worker(run_id: str, item: dict[str, Any]) -> None:
    try:
        while not item["stop"].is_set():
            with _leader_follower_lock:
                if _leader_follower_runs.get(run_id) is not item:
                    return
                ctx = dict(item["ctx"])
            try:
                result = _leader_follower_step(item, ctx)
            except Exception as exc:
                result = _leader_follower_result(running=True, armed=bool(ctx.get("armed")), report=f"leader-follower FAILED: {exc}; commands suppressed.")
            with _leader_follower_lock:
                if _leader_follower_runs.get(run_id) is item:
                    item["last"] = result
                    item["updated_at"] = time.time()
            hz = max(1.0, min(60.0, float(ctx.get("loop_hz") or 60.0)))
            item["stop"].wait(1.0 / hz)
    finally:
        rb.release_joint_stream(item.get("leader_session"))
        rb.release_joint_stream(item.get("follower_session"))


def run_leader_follower(ctx: dict) -> dict:
    """Handle the start/stop/check action dispatch for one leader-follower run."""
    action = str(ctx.get("action") or "start").strip().lower()
    run_id = str(ctx.get("run_id") or "leader_follower").strip() or "leader_follower"
    if action == "stop":
        stopped = _stop_leader_follower(run_id)
        return _leader_follower_result(report=f"leader-follower '{run_id}' {'stopped' if stopped else 'was not running'}.")
    with _leader_follower_lock:
        existing = _leader_follower_runs.get(run_id)
        if existing is not None and not existing["thread"].is_alive():
            _leader_follower_runs.pop(run_id, None)
            sample_stream.unregister(run_id)
            existing = None
    if action == "check":
        if existing is None:
            return _leader_follower_result(report=f"leader-follower '{run_id}' is not running.")
        with _leader_follower_lock:
            latest = dict(existing.get("last") or _leader_follower_result(running=True, armed=bool(ctx.get("armed"))))
            latest["sample_stream"] = dict(existing.get("sample_stream") or {})
            return latest
    if existing is not None:
        with _leader_follower_lock:
            existing["ctx"] = dict(ctx)
            latest = dict(existing.get("last") or _leader_follower_result(running=True, armed=bool(ctx.get("armed"))))
        latest["running"] = True
        latest["sample_stream"] = dict(existing.get("sample_stream") or {})
        return latest
    item: dict[str, Any] = {
        "ctx": dict(ctx), "stop": threading.Event(), "leader_session": None,
        "follower_session": None, "leader_signature": None, "follower_signature": None,
        "last": _leader_follower_result(running=True, armed=bool(ctx.get("armed")), report="Waiting for both robots…"),
        "updated_at": time.time(),
        "sample": {}, "sample_sequence": 0,
    }
    item["sample_stream"] = sample_stream.register(run_id, lambda: dict(item.get("sample") or {}))
    thread = threading.Thread(target=_leader_follower_worker, args=(run_id, item), name=f"blacknode-leader-follower-{run_id}", daemon=True)
    item["thread"] = thread
    with _leader_follower_lock:
        _leader_follower_runs[run_id] = item
    thread.start()
    result = dict(item["last"])
    result["sample_stream"] = dict(item["sample_stream"])
    return result
