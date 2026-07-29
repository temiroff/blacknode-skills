"""ROS 2 follow adapter: visual-follow and leader/follower node contracts.

Moved out of blacknode-ros2's ``ros2_live.py`` along with the node
implementations themselves (see the embodied-robotics-roadmap Stage D item on
extracting follow-target/leader-follower behavior into this adapter).
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import pytest

import blacknode  # noqa: F401 - discover extension packages
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb

_SKILLS_ROOT = Path(__file__).resolve().parents[1] / "components" / "follow"
_NODES = _SKILLS_ROOT / "nodes"
_ADAPTER_NODES = _SKILLS_ROOT / "adapters" / "ros2" / "nodes"
_before = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_skills.follow", _NODES)
_import_nodes_module("blacknode.pkg.blacknode_skills.follow.adapters.ros2", _ADAPTER_NODES)
_tag_new_package_nodes(_before, "blacknode-skills", _ADAPTER_NODES, "follow", "ros2")

from blacknode.pkg.blacknode_skills.follow import (
    follow_runtime,
    leader_follower_runtime,
    leader_subscription_runtime,
)


def test_follow_person_ros2_nodes_registered_with_category():
    for name in [
        "ROS2NativeFollowDetectionJoint",
        "ROS2FollowDetectionJoint",
        "RobotFollow",
        "ROS2LeaderFollower",
        "ROS2PublishJointState",
        "ROS2SubscribeJointState",
        "ROS2JointController",
    ]:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "Skills"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-skills"
    assert _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]._bn_hidden is True
    assert _NODE_REGISTRY["ROS2PublishJointState"]._bn_hidden is False
    assert _NODE_REGISTRY["ROS2SubscribeJointState"]._bn_hidden is False
    assert _NODE_REGISTRY["ROS2JointController"]._bn_hidden is False
    assert _NODE_REGISTRY["ROS2PublishJointState"]._bn_live_capable is True
    assert _NODE_REGISTRY["ROS2SubscribeJointState"]._bn_live_capable is True
    assert _NODE_REGISTRY["ROS2JointController"]._bn_live_capable is True
    for legacy_name in (
        "ROS2JointStatePublish",
        "ROS2JointSubscribe",
        "ROS2JointReplicate",
        "ROS2JointPublish",
        "ROS2LeaderJointSubscriber",
        "ROS2FollowerJointPublisher",
    ):
        assert legacy_name not in _NODE_REGISTRY


def test_managed_leader_subscription_exposes_a_fresh_stream(monkeypatch):
    class FakeSession:
        def snapshot(self):
            return (
                {"shoulder_pan": 0.25},
                {"torque_enabled": False},
                0.01,
            )

    released = []
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "acquire_joint_stream", lambda *args, **kwargs: FakeSession())
    monkeypatch.setattr(
        nr,
        "release_joint_stream",
        lambda session, **kwargs: released.append(session),
    )
    leader_subscription_runtime.stop_leader_subscription_services()
    try:
        result = _NODE_REGISTRY["ROS2SubscribeJointState"]({
            "action": "start",
            "run_id": "test_joint_subscription",
            "transport": "native",
            "state_topic": "/leader/joint_states",
            "config_topic": "/leader/joint_config",
        })
        assert result["running"] is True
        assert result["live"] is True
        assert result["subscription"]["run_id"] == "test_joint_subscription"
        assert result["pose"]["shoulder_pan"] == pytest.approx(math.degrees(0.25))
    finally:
        leader_subscription_runtime.stop_leader_subscription_services()
    assert released


def test_split_follower_publisher_consumes_managed_subscription(monkeypatch):
    class FollowerSession:
        def __init__(self):
            self.published = []

        def snapshot(self):
            return (
                {"shoulder_pan": 0.0},
                {
                    "torque_enabled": True,
                    "commands_allowed": True,
                    "joints": {"shoulder_pan": {"lower": -1.0, "upper": 1.0}},
                },
                0.01,
            )

        def wait_for_pose(self, timeout):
            return {"shoulder_pan": 0.0}

        def wait_for_config(self, timeout):
            return {}

        def publish(self, pose):
            self.published.append(dict(pose))

    follower = FollowerSession()
    monkeypatch.setattr(leader_follower_runtime, "_resolve_transport", lambda ctx: "native")
    monkeypatch.setattr(
        leader_subscription_runtime,
        "subscription_snapshot",
        lambda subscription: {
            "state_topic": "/leader/joint_states",
            "config_topic": "/leader/joint_config",
            "pose": {"shoulder_pan": 0.25},
            "config": {"torque_enabled": False},
            "age": 0.01,
            "hardware_id": "LEADER",
            "calibration_path": "leader.json",
        },
    )
    monkeypatch.setattr(nr, "acquire_joint_stream", lambda *args, **kwargs: follower)
    monkeypatch.setattr(nr, "release_joint_stream", lambda *args, **kwargs: None)
    item = {
        "leader_session": None, "follower_session": None,
        "leader_signature": None, "follower_signature": None,
    }
    result = leader_follower_runtime._leader_follower_step(item, {
        "armed": True,
        "leader_subscription": {"run_id": "leader"},
        "follower_robot": {
            "state_topic": "/follower/joint_states",
            "command_topic": "/follower/joint_commands",
            "config_topic": "/follower/joint_config",
            "driver": {
                "hardware_id": "FOLLOWER",
                "calibration_path": "follower.json",
            },
        },
        "require_calibration": True,
        "require_leader_released": True,
        "tracking_mode": "direct",
    })
    assert result["commanded"] is True
    assert follower.published
    assert item["leader_session"] is None


def test_joint_state_publisher_exposes_running_robot_publication():
    result = _NODE_REGISTRY["ROS2PublishJointState"]({
        "robot": {
            "ready": True,
            "state_topic": "/leader/joint_states",
            "driver": {
                "running": True,
                "run_id": "leader_driver",
                "transport": "native",
            },
        },
        "state_topic": "/leader/joint_states",
    })

    assert result["publishing"] is True
    assert result["state_topic"] == "/leader/joint_states"
    assert result["publisher"]["message_type"] == "sensor_msgs/msg/JointState"
    assert result["publisher"]["robot_run_id"] == "leader_driver"


def test_joint_state_publisher_rejects_topic_mismatch():
    result = _NODE_REGISTRY["ROS2PublishJointState"]({
        "robot": {
            "ready": True,
            "state_topic": "/other/joint_states",
            "driver": {"running": True},
        },
        "state_topic": "/leader/joint_states",
    })

    assert result["publishing"] is False
    assert "topic mismatch" in result["report"]


def test_generic_joint_replicator_exposes_role_neutral_ports(monkeypatch):
    captured = {}

    def run(ctx):
        captured.update(ctx)
        return {
            "running": True,
            "live": True,
            "armed": True,
            "commanded": True,
            "leader_pose": {"joint": 1.0},
            "follower_pose": {"joint": 0.5},
            "target": {"joint": 1.0},
            "sample_stream": {"stream_id": "joint"},
            "clamped": [],
            "joint_count": 1,
            "dashboard": "",
            "summary": {},
            "report": "published",
        }

    monkeypatch.setattr(leader_follower_runtime, "run_leader_follower", run)
    result = _NODE_REGISTRY["ROS2JointController"]({
        "subscription": {"run_id": "source"},
        "robot": {"command_topic": "/joint_commands"},
        "armed": True,
    })

    assert captured["leader_subscription"] == {"run_id": "source"}
    assert captured["follower_robot"]["command_topic"] == "/joint_commands"
    assert result["published"] is True
    assert result["source_pose"] == {"joint": 1.0}
    assert result["current_pose"] == {"joint": 0.5}
    assert result["command"] == {"joint": 1.0}
    assert result["message_stream"] == {"stream_id": "joint"}

def test_native_follow_detection_joint_blocked_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("native ROS 2 must not be touched while disarmed")

    monkeypatch.setattr(nr, "read_pose", fail_if_called)
    monkeypatch.setattr(nr, "stream_motion", fail_if_called)

    result = _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "armed": False,
    })

    assert result["report"].startswith("BLOCKED:")
    assert result["moved"] is False
    assert result["command"] > 0


def test_native_follow_detection_joint_streams_toward_center(monkeypatch):
    start = {"shoulder_pan": math.radians(10.0), "elbow": 0.0}
    after = {"shoulder_pan": math.radians(20.0), "elbow": 0.0}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: next(poses))

    def fake_stream(command_topic, names, s, t, **kwargs):
        captured["command_topic"] = command_topic
        captured["names"] = names
        captured["target"] = t
        return {"ok": True, "sent": 12}

    monkeypatch.setattr(nr, "stream_motion", fake_stream)

    result = _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "robot": {"state_topic": "/state", "command_topic": "/cmd"},
        "target_x": 0.5,
        "gain": 40.0,
        "max_step": 15.0,
        "units": "degrees",
        "armed": True,
    })

    assert result["moved"] is True
    assert math.isclose(result["command"], 10.0, abs_tol=1e-6)
    assert captured["command_topic"] == "/cmd"
    assert captured["names"] == ["shoulder_pan", "elbow"]
    assert math.isclose(captured["target"]["shoulder_pan"], math.radians(20.0), abs_tol=1e-6)
    assert "native follow shoulder_pan" in result["report"]


def test_native_follow_detection_joint_structured_error_without_rclpy(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "rclpy is not importable"))

    follow = _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "armed": True,
    })
    assert follow["moved"] is False
    assert "rclpy is not importable" in follow["report"]


def test_follow_detection_joint_blocked_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("rosbridge must not be touched while disarmed")

    monkeypatch.setattr(rb, "get_connection", fail_if_called)
    monkeypatch.setattr(rb, "read_pose", fail_if_called)
    monkeypatch.setattr(rb, "stream_motion", fail_if_called)
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "armed": False,
    })
    assert result["report"].startswith("BLOCKED:")
    assert result["moved"] is False
    assert result["command"] > 0


def test_follow_detection_joint_streams_toward_center(monkeypatch):
    start = {"shoulder_pan": math.radians(10.0), "elbow": 0.0}
    after = {"shoulder_pan": math.radians(20.0), "elbow": 0.0}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: next(poses))

    def fake_stream(host, port, command_topic, names, s, t, **kwargs):
        captured["host"] = host
        captured["command_topic"] = command_topic
        captured["names"] = names
        captured["target"] = t
        return {"ok": True, "sent": 12}

    monkeypatch.setattr(rb, "stream_motion", fake_stream)
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "robot": {"host": "robot.local", "port": 9090, "state_topic": "/state", "command_topic": "/cmd"},
        "target_x": 0.5,
        "gain": 40.0,
        "max_step": 15.0,
        "units": "degrees",
        "armed": True,
    })
    assert result["moved"] is True
    assert math.isclose(result["command"], 10.0, abs_tol=1e-6)
    assert captured["host"] == "robot.local"
    assert captured["command_topic"] == "/cmd"
    assert math.isclose(captured["target"]["shoulder_pan"], math.radians(20.0), abs_tol=1e-6)
    assert captured["names"] == ["shoulder_pan", "elbow"]
    assert "cube zone=LEFT, x=160.0/640" in result["report"]


def test_follow_detection_uses_payload_frame_width_and_reports_zone(monkeypatch):
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: pytest.fail("disarmed preview must not read ROS"))
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 455}, "frame_width": 640},
        "frame_width": 960,
        "target_x": 0.5,
        "deadband": 0.16,
        "gain": 10.0,
        "max_step": 2.0,
        "invert": True,
        "armed": False,
    })

    assert result["command"] == 2.0
    assert "zone=RIGHT, x=455.0/640" in result["report"]


def test_follow_detection_joint_noops_inside_deadband(monkeypatch):
    def fail_if_streamed(*args, **kwargs):
        raise AssertionError("must not stream commands inside deadband")

    monkeypatch.setattr(rb, "stream_motion", fail_if_streamed)
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 322}},
        "frame_width": 640,
        "target_x": 0.5,
        "deadband": 0.02,
        "armed": True,
    })
    assert result["moved"] is False
    assert result["command"] == 0.0
    assert "centered enough" in result["report"]


def test_continuous_follow_runs_until_stopped(monkeypatch):
    called = threading.Event()

    def fake_follow(item, ctx):
        called.set()
        return {
            "moved": True,
            "joint": ctx["joint"],
            "before": {ctx["joint"]: 0.0},
            "after": {ctx["joint"]: 2.0},
            "target": {ctx["joint"]: 2.0},
            "error": 0.25,
            "command": 2.0,
            "report": "MOVE LEFT",
        }

    monkeypatch.setattr(follow_runtime, "_continuous_follow_step", fake_follow)
    follow_runtime.stop_continuous_follow_services()
    try:
        started = _NODE_REGISTRY["RobotFollow"]({
            "action": "start",
            "run_id": "test_follow",
            "loop_hz": 20.0,
            "detection_url": "http://127.0.0.1:9999/detection.json",
            "joint": "shoulder_pan",
            "armed": True,
        })
        assert started["running"] is True
        assert called.wait(1.0)

        checked = _NODE_REGISTRY["RobotFollow"]({
            "action": "check",
            "run_id": "test_follow",
            "joint": "shoulder_pan",
        })
        assert checked["running"] is True
        assert checked["command"] == 2.0

        stopped = _NODE_REGISTRY["RobotFollow"]({
            "action": "stop",
            "run_id": "test_follow",
            "joint": "shoulder_pan",
        })
        assert stopped["running"] is False
        assert "stopped" in stopped["report"]
        assert follow_runtime.continuous_follow_runtime_status() == []
    finally:
        follow_runtime.stop_continuous_follow_services()


def test_continuous_follow_step_reuses_persistent_joint_stream(monkeypatch):
    class FakeSession:
        def __init__(self):
            self.published = []

        def snapshot(self):
            return (
                {"shoulder_pan": 0.0, "elbow": 0.25},
                {
                    "commands_allowed": True,
                    "joints": {
                        "shoulder_pan": {
                            "lower": -1.0,
                            "upper": 1.0,
                        },
                        "elbow": {
                            "lower": -1.0,
                            "upper": 1.0,
                        },
                    },
                },
                0.01,
            )

        def wait_for_pose(self, timeout):
            return {"shoulder_pan": 0.0, "elbow": 0.25}

        def publish(self, pose):
            self.published.append(pose)

    session = FakeSession()
    acquired = []
    monkeypatch.setattr(follow_runtime, "_read_detection_url", lambda *a, **k: ({
        "found": True,
        "updated_at": time.time(),
        "detection": {"found": True, "center": {"x": 100}, "frame_width": 640},
    }, ""))
    monkeypatch.setattr(rb, "acquire_joint_stream", lambda *a, **k: acquired.append((a, k)) or session)
    monkeypatch.setattr(rb, "release_joint_stream", lambda _session: None)
    item = {"session": None, "session_signature": None}
    ctx = {
        "joint": "shoulder_pan",
        "units": "degrees",
        "detection_stream": {"url": "http://detector/detection.json", "stream_id": "cube"},
        "loop_hz": 10.0,
        "gain": 10.0,
        "max_step": 2.0,
        "armed": True,
    }

    first = follow_runtime._continuous_follow_step(item, ctx)
    second = follow_runtime._continuous_follow_step(item, ctx)

    assert first["running"] is True
    assert second["running"] is True
    assert len(acquired) == 1
    assert len(session.published) == 2
    assert set(session.published[0]) == {"shoulder_pan", "elbow"}
    # Motion safety does not let the desired target run farther ahead while
    # hardware feedback remains unchanged.
    assert session.published[1]["shoulder_pan"] == session.published[0]["shoulder_pan"]


def test_continuous_follow_step_resets_stale_joint_stream(monkeypatch):
    class FakeSession:
        def snapshot(self):
            return ({"shoulder_pan": 0.0}, {}, 99.0)

        def wait_for_pose(self, timeout):
            return {"shoulder_pan": 0.0}

    session = FakeSession()
    released = []
    monkeypatch.setattr(follow_runtime, "_read_detection_url", lambda *a, **k: ({
        "found": True,
        "updated_at": time.time(),
        "detection": {"found": True, "center": {"x": 100}, "frame_width": 640},
    }, ""))
    monkeypatch.setattr(
        rb,
        "release_joint_stream",
        lambda session, **kwargs: released.append((session, kwargs)),
    )
    signature = ("127.0.0.1", 9090, "/joint_states", "/joint_commands", "")
    item = {"session": session, "session_signature": signature, "session_resets": 0}
    ctx = {
        "joint": "shoulder_pan",
        "units": "degrees",
        "detection_stream": {"url": "http://detector/detection.json", "stream_id": "cube"},
        "loop_hz": 10.0,
        "gain": 10.0,
        "max_step": 2.0,
        "armed": True,
    }

    result = follow_runtime._continuous_follow_step(item, ctx)

    assert result["running"] is False
    assert "resetting subscription" in result["report"]
    assert released == [(session, {"discard": True})]
    assert item["session"] is None
    assert item["session_signature"] is None
    assert item["session_resets"] == 1


def test_continuous_follow_disarmed_does_not_start():
    follow_runtime.stop_continuous_follow_services()
    result = _NODE_REGISTRY["RobotFollow"]({
        "action": "start",
        "run_id": "test_disarmed",
        "detection_url": "http://127.0.0.1:9999/detection.json",
        "joint": "shoulder_pan",
        "armed": False,
    })
    assert result["running"] is False
    assert result["report"].startswith("BLOCKED:")
    assert follow_runtime.continuous_follow_runtime_status() == []


def test_leader_follower_previews_disarmed_and_commands_bounded_targets(monkeypatch):
    class FakeSession:
        def __init__(self, pose, config):
            self.pose = pose
            self.config = config
            self.published = []

        def snapshot(self):
            return self.pose, self.config, 0.01

        def wait_for_pose(self, timeout):
            return self.pose

        def wait_for_config(self, timeout):
            return self.config

        def publish(self, pose):
            self.published.append(dict(pose))

    leader_session = FakeSession(
        {"shoulder_pan": math.radians(30.0), "elbow_flex": math.radians(-20.0)},
        {"torque_enabled": False},
    )
    follower_session = FakeSession(
        {"shoulder_pan": 0.0, "elbow_flex": 0.0},
        {
            "commands_allowed": True,
            "joints": {
                "shoulder_pan": {"lower": math.radians(-90), "upper": math.radians(90)},
                "elbow_flex": {"lower": math.radians(-90), "upper": math.radians(90)},
            },
        },
    )
    acquired = []

    def acquire(*signature, **kwargs):
        acquired.append(signature)
        return leader_session if signature[2].startswith("/leader/") else follower_session

    monkeypatch.setattr(leader_follower_runtime, "_resolve_transport", lambda ctx: "rosbridge")
    monkeypatch.setattr(rb, "acquire_joint_stream", acquire)
    monkeypatch.setattr(rb, "release_joint_stream", lambda *args, **kwargs: None)
    item = {
        "leader_session": None, "follower_session": None,
        "leader_signature": None, "follower_signature": None,
    }
    base_ctx = {
        "leader_robot": {
            "state_topic": "/leader/joint_states", "command_topic": "/leader/joint_commands",
            "config_topic": "/leader/joint_config",
            "driver": {"hardware_id": "LEADER-1", "calibration_path": "leader.json"},
        },
        "follower_robot": {
            "state_topic": "/follower/joint_states", "command_topic": "/follower/joint_commands",
            "config_topic": "/follower/joint_config",
            "driver": {"hardware_id": "FOLLOWER-1", "calibration_path": "follower.json"},
        },
        "max_step_deg": 2.0,
        "deadband_deg": 0.1,
        "tracking_mode": "bounded",
        "require_calibration": True,
        "require_leader_released": True,
    }

    preview = leader_follower_runtime._leader_follower_step(item, {**base_ctx, "armed": False})
    commanded = leader_follower_runtime._leader_follower_step(item, {**base_ctx, "armed": True})

    assert preview["running"] is True
    assert preview["live"] is True
    assert preview["armed"] is False
    assert follower_session.published and commanded["commanded"] is True
    assert commanded["target"]["shoulder_pan"] == pytest.approx(2.0)
    assert commanded["target"]["elbow_flex"] == pytest.approx(-2.0)
    assert item["sample"]["kind"] == "blacknode.teleoperation-sample"
    assert item["sample"]["joint_names"] == ["shoulder_pan", "elbow_flex"]
    assert item["sample"]["units"] == "radians"
    assert item["sample"]["observation"]["shoulder_pan"] == pytest.approx(0.0)
    assert item["sample"]["action"]["shoulder_pan"] == pytest.approx(math.radians(2.0))
    assert len(acquired) == 2
    assert acquired[0][3] == ""
    assert acquired[1][3] == "/follower/joint_commands"

    direct = leader_follower_runtime._leader_follower_step(item, {
        **base_ctx,
        "armed": True,
        "tracking_mode": "direct",
    })
    assert direct["target"]["shoulder_pan"] == pytest.approx(30.0)
    assert direct["target"]["elbow_flex"] == pytest.approx(-20.0)
    assert "Direct-following" in direct["report"]


def test_leader_follower_blocks_same_usb_device(monkeypatch):
    result = leader_follower_runtime._leader_follower_step({}, {
        "armed": True,
        "leader_robot": {"driver": {"hardware_id": "SAME", "calibration_path": "a.json"}},
        "follower_robot": {"driver": {"hardware_id": "SAME", "calibration_path": "b.json"}},
    })

    assert result["commanded"] is False
    assert "same USB device" in result["report"]


def test_leader_follower_uses_native_joint_streams(monkeypatch):
    class FakeSession:
        def __init__(self, pose, config):
            self.pose = pose
            self.config = config
            self.published = []

        def snapshot(self):
            return self.pose, self.config, 0.01

        def wait_for_pose(self, timeout):
            return self.pose

        def wait_for_config(self, timeout):
            return self.config

        def publish(self, pose):
            self.published.append(dict(pose))

    leader_session = FakeSession(
        {"shoulder_pan": 0.25},
        {"torque_enabled": False},
    )
    follower_session = FakeSession(
        {"shoulder_pan": 0.0},
        {
            "commands_allowed": True,
            "joints": {"shoulder_pan": {"lower": -1.0, "upper": 1.0}},
        },
    )
    acquired = []

    def acquire(*signature, **kwargs):
        acquired.append((signature, kwargs))
        return leader_session if signature[0].startswith("/leader/") else follower_session

    monkeypatch.setattr(leader_follower_runtime, "_resolve_transport", lambda ctx: "native")
    monkeypatch.setattr(nr, "acquire_joint_stream", acquire)
    monkeypatch.setattr(nr, "release_joint_stream", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        rb,
        "acquire_joint_stream",
        lambda *args, **kwargs: pytest.fail("native transport must not open rosbridge"),
    )
    item = {
        "leader_session": None,
        "follower_session": None,
        "leader_signature": None,
        "follower_signature": None,
        "leader_transport": None,
        "follower_transport": None,
    }

    result = leader_follower_runtime._leader_follower_step(item, {
        "armed": False,
        "transport": "auto",
        "placement": "same_device",
        "follower_robot": {
            "state_topic": "/follower/joint_states",
            "command_topic": "/follower/joint_commands",
            "config_topic": "/follower/joint_config",
            "driver": {
                "hardware_id": "FOLLOWER",
                "calibration_path": "follower.json",
            },
        },
        "require_calibration": True,
        "require_leader_released": True,
        "tracking_mode": "direct",
    })

    assert result["live"] is True
    assert result["commanded"] is False
    assert acquired == [
        (
            ("/leader/joint_states", "", "/leader/joint_config"),
            {
                "timeout": 2.0,
                "node_name": "blacknode_leader_follower_leader",
            },
        ),
        (
            (
                "/follower/joint_states",
                "/follower/joint_commands",
                "/follower/joint_config",
            ),
            {
                "timeout": 2.0,
                "node_name": "blacknode_leader_follower_follower",
            },
        ),
    ]
    assert item["leader_transport"] == "native"
    assert item["follower_transport"] == "native"


def test_leader_follower_arming_controls_only_follower_torque(monkeypatch):
    class FakeSession:
        def __init__(self, pose, config):
            self.pose = pose
            self.config = config
            self.published = []

        def snapshot(self):
            return self.pose, self.config, 0.01

        def wait_for_pose(self, timeout):
            return self.pose

        def wait_for_config(self, timeout):
            return self.config

        def publish(self, pose):
            self.published.append(dict(pose))

    leader_session = FakeSession(
        {"shoulder_pan": 0.25},
        {"torque_enabled": False},
    )
    follower_session = FakeSession(
        {"shoulder_pan": 0.0},
        {
            "torque_enabled": False,
            "commands_allowed": False,
            "last_error": "",
            "joints": {"shoulder_pan": {"lower": -1.0, "upper": 1.0}},
        },
    )
    control_actions = []

    def acquire(*signature, **kwargs):
        return (
            leader_session
            if signature[0].startswith("/leader/")
            else follower_session
        )

    def publish_string(topic, payload, timeout):
        action = json.loads(payload)["action"]
        control_actions.append((topic, action))
        enabled = action == "exit_teach"
        follower_session.config["torque_enabled"] = enabled
        follower_session.config["commands_allowed"] = enabled
        return {"ok": True}

    monkeypatch.setattr(
        leader_follower_runtime,
        "_resolve_transport",
        lambda ctx: "native",
    )
    monkeypatch.setattr(nr, "acquire_joint_stream", acquire)
    monkeypatch.setattr(nr, "release_joint_stream", lambda *args, **kwargs: None)
    monkeypatch.setattr(nr, "publish_string", publish_string)
    item = {
        "leader_session": None,
        "follower_session": None,
        "leader_signature": None,
        "follower_signature": None,
        "leader_transport": None,
        "follower_transport": None,
    }
    ctx = {
        "armed": True,
        "transport": "auto",
        "follower_robot": {
            "state_topic": "/follower/joint_states",
            "command_topic": "/follower/joint_commands",
            "config_topic": "/follower/joint_config",
            "control_topic": "/follower/robot_control",
            "driver": {
                "hardware_id": "FOLLOWER",
                "calibration_path": "follower.json",
            },
        },
        "require_calibration": True,
        "require_leader_released": True,
        "tracking_mode": "direct",
    }

    arming = leader_follower_runtime._leader_follower_step(item, ctx)
    following = leader_follower_runtime._leader_follower_step(item, ctx)
    disarming = leader_follower_runtime._leader_follower_step(
        item,
        {**ctx, "armed": False},
    )

    assert arming["commanded"] is False
    assert arming["report"].startswith("ARMING:")
    assert following["commanded"] is True
    assert follower_session.published
    assert disarming["commanded"] is False
    assert disarming["report"].startswith("DISARMING:")
    assert control_actions == [
        ("/follower/robot_control", "exit_teach"),
        ("/follower/robot_control", "enter_teach"),
    ]
    assert leader_session.config["torque_enabled"] is False


def test_remote_leader_uses_separate_rosbridge_endpoint(monkeypatch):
    class FakeSession:
        def __init__(self, pose, config):
            self.pose = pose
            self.config = config
            self.published = []

        def snapshot(self):
            return self.pose, self.config, 0.01

        def wait_for_pose(self, timeout):
            return self.pose

        def wait_for_config(self, timeout):
            return self.config

        def publish(self, pose):
            self.published.append(dict(pose))

    leader_session = FakeSession(
        {"shoulder_pan": 0.25},
        {"torque_enabled": False},
    )
    follower_session = FakeSession(
        {"shoulder_pan": 0.0},
        {
            "commands_allowed": True,
            "joints": {"shoulder_pan": {"lower": -1.0, "upper": 1.0}},
        },
    )
    acquired = []

    def acquire(*signature, **kwargs):
        acquired.append(signature)
        return leader_session if signature[0] == "leader.local" else follower_session

    monkeypatch.setattr(leader_follower_runtime, "_resolve_transport", lambda ctx: "rosbridge")
    monkeypatch.setattr(rb, "acquire_joint_stream", acquire)
    monkeypatch.setattr(rb, "release_joint_stream", lambda *args, **kwargs: None)
    item = {
        "leader_session": None,
        "follower_session": None,
        "leader_signature": None,
        "follower_signature": None,
    }
    result = leader_follower_runtime._leader_follower_step(item, {
        "armed": True,
        "placement": "separate_devices",
        "leader_host": "leader.local",
        "leader_port": 9091,
        "follower_host": "127.0.0.1",
        "follower_port": 9090,
        "follower_robot": {
            "state_topic": "/follower/joint_states",
            "command_topic": "/follower/joint_commands",
            "config_topic": "/follower/joint_config",
            "driver": {
                "hardware_id": "FOLLOWER",
                "profile": {
                    "calibration": {"hardware_id": "FOLLOWER"},
                },
            },
        },
        "require_calibration": True,
        "require_leader_released": True,
        "tracking_mode": "direct",
    })

    assert result["commanded"] is True
    assert acquired[0][:2] == ("leader.local", 9091)
    assert acquired[0][3] == ""
    assert acquired[1][:2] == ("127.0.0.1", 9090)
    assert follower_session.published


def test_leader_follower_resets_stale_side_and_recovers(monkeypatch):
    class FakeSession:
        def __init__(self, pose, config, age):
            self.pose = pose
            self.config = config
            self.age = age
            self.published = []

        def snapshot(self):
            return self.pose, self.config, self.age

        def wait_for_pose(self, timeout):
            return self.pose

        def wait_for_config(self, timeout):
            return self.config

        def publish(self, pose):
            self.published.append(pose)

    leader_config = {"torque_enabled": False}
    follower_config = {
        "commands_allowed": True,
        "joints": {"shoulder_pan": {"lower": -1.0, "upper": 1.0}},
    }
    stale_leader = FakeSession({"shoulder_pan": 0.4}, leader_config, 99.0)
    fresh_leader = FakeSession({"shoulder_pan": 0.4}, leader_config, 0.01)
    follower = FakeSession({"shoulder_pan": 0.0}, follower_config, 0.01)
    leader_sessions = [stale_leader, fresh_leader]
    released = []

    def acquire(*signature, **kwargs):
        return leader_sessions.pop(0) if signature[2].startswith("/leader/") else follower

    def release(session, *, discard=False):
        if session is not None:
            released.append((session, discard))

    monkeypatch.setattr(leader_follower_runtime, "_resolve_transport", lambda ctx: "rosbridge")
    monkeypatch.setattr(rb, "acquire_joint_stream", acquire)
    monkeypatch.setattr(rb, "release_joint_stream", release)
    item = {
        "leader_session": None, "follower_session": None,
        "leader_signature": None, "follower_signature": None,
    }
    ctx = {
        "armed": True,
        "tracking_mode": "direct",
        "stale_after": 0.75,
        "require_calibration": True,
        "require_leader_released": True,
        "leader_robot": {
            "state_topic": "/leader/joint_states", "command_topic": "/leader/joint_commands",
            "config_topic": "/leader/joint_config",
            "driver": {"hardware_id": "LEADER", "calibration_path": "leader.json"},
        },
        "follower_robot": {
            "state_topic": "/follower/joint_states", "command_topic": "/follower/joint_commands",
            "config_topic": "/follower/joint_config",
            "driver": {"hardware_id": "FOLLOWER", "calibration_path": "follower.json"},
        },
    }

    waiting = leader_follower_runtime._leader_follower_step(item, ctx)
    recovered = leader_follower_runtime._leader_follower_step(item, ctx)

    assert waiting["commanded"] is False
    assert "leader stale (99.00s > 0.75s)" in waiting["report"]
    assert "resetting subscription" in waiting["report"]
    assert item["leader_session_resets"] == 1
    assert (stale_leader, True) in released
    assert recovered["live"] is True
    assert recovered["commanded"] is True
    assert "Direct-following" in recovered["report"]
    assert follower.published


def test_leader_follower_live_service_updates_and_stops(monkeypatch):
    called = threading.Event()

    def fake_step(item, ctx):
        called.set()
        return leader_follower_runtime._leader_follower_result(
            running=True,
            live=True,
            armed=bool(ctx.get("armed")),
            report="test leader-follower tick",
        )

    monkeypatch.setattr(leader_follower_runtime, "_leader_follower_step", fake_step)
    leader_follower_runtime.stop_leader_follower_services()
    try:
        started = _NODE_REGISTRY["ROS2LeaderFollower"]({
            "action": "start",
            "run_id": "test_leader_follower",
            "loop_hz": 20.0,
            "armed": False,
        })
        assert started["running"] is True
        assert called.wait(1.0)

        updated = leader_follower_runtime.update_leader_follower_config("test_leader_follower", {"armed": True})
        assert updated["ok"] is True
        assert leader_follower_runtime.leader_follower_runtime_status()[0]["armed"] is True

        stopped = _NODE_REGISTRY["ROS2LeaderFollower"]({
            "action": "stop",
            "run_id": "test_leader_follower",
        })
        assert stopped["running"] is False
        assert "stopped" in stopped["report"]
        assert leader_follower_runtime.leader_follower_runtime_status() == []
    finally:
        leader_follower_runtime.stop_leader_follower_services()


def test_leader_follower_remote_control_is_explicit_and_scoped():
    run_id = "SO Arm Follower"
    item = {"ctx": {"armed": False}}
    with leader_follower_runtime._leader_follower_lock:
        leader_follower_runtime._leader_follower_runs[run_id] = item
    try:
        assert leader_follower_runtime.leader_follower_control_topic(
            run_id,
        ) == "/blacknode/leader_follower/SO_Arm_Follower/control"
        assert leader_follower_runtime.leader_follower_control_topic(
            run_id,
            "/blacknode/leader_follower/follower/control;unsafe",
        ) == "/blacknode/leader_follower/SO_Arm_Follower/control"
        assert leader_follower_runtime._apply_control_message(
            run_id,
            item,
            '{"armed": true}',
        )
        assert item["ctx"]["armed"] is True
        assert not leader_follower_runtime._apply_control_message(
            run_id,
            item,
            '{"action": "publish arbitrary command"}',
        )
        assert item["ctx"]["armed"] is True
    finally:
        with leader_follower_runtime._leader_follower_lock:
            leader_follower_runtime._leader_follower_runs.pop(run_id, None)


def test_joint_control_stop_delegates_to_follow_person_runtimes(monkeypatch):
    """The arm adapter's stop_runtime_services() reaches this adapter's run counts."""
    from blacknode.packages import _import_nodes_module
    from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as ros2_rb

    adapter_nodes = (
        Path(__file__).resolve().parents[2]
        / "blacknode-motion" / "components" / "arm" / "adapters" / "ros2" / "nodes"
    )
    _import_nodes_module(
        "blacknode.pkg.blacknode_motion.arm.adapters.ros2", adapter_nodes
    )
    from blacknode.pkg.blacknode_motion.arm.adapters.ros2 import joint_motion as live

    monkeypatch.setattr(follow_runtime, "stop_continuous_follow_services", lambda: {
        "ok": True, "stopped": 2, "report": "stopped 2",
    })
    monkeypatch.setattr(leader_follower_runtime, "stop_leader_follower_services", lambda: {
        "ok": True, "stopped": 1, "report": "stopped 1",
    })
    monkeypatch.setattr(live, "_teach_monitors", {})
    monkeypatch.setattr(ros2_rb, "close_joint_streams", lambda: 1)

    result = live.stop_runtime_services()

    assert result["stopped"]["managed_runs"] == 3
    assert result["stopped"]["joint_streams"] == 1
    assert "2 follow controller(s)" in result["report"]
    assert "1 leader-follower controller(s)" in result["report"]
