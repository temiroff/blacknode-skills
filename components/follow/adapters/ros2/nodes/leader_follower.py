"""ROS 2 leader/follower teleoperation node.

Delegates its run lifecycle to
:mod:`blacknode.pkg.blacknode_skills.follow.leader_follower_runtime`
so Stop All and status reporting can reach it without importing this
node-decorated module.
"""
from __future__ import annotations

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

from blacknode.pkg.blacknode_skills.follow import leader_follower_runtime
from blacknode.pkg.blacknode_skills.follow import leader_subscription_runtime

_CATEGORY = "Skills"


@node(
    name="ROS2LeaderFollower",
    live=True,
    category=_CATEGORY,
    description="Stream a released leader robot pose into a separately calibrated follower. Defaults match LeRobot: direct targets at 60 Hz without a deadband; calibrated limits and stale-data suppression remain enforced.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop", "check"], default="start"),
        "run_id": Text(default="leader_follower"),
        "control_topic": Text(default=""),
        "leader_robot": Dict,
        "follower_robot": Dict,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "placement": Enum(["same_device", "separate_devices"], default="same_device"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "leader_host": Text(default=""),
        "leader_port": Int(default=0),
        "follower_host": Text(default=""),
        "follower_port": Int(default=0),
        "joint_map": Dict,
        "scale": Dict,
        "offset_deg": Dict,
        "tracking_mode": Enum(["bounded", "direct"], default="direct"),
        "loop_hz": Float(default=60.0),
        "max_step_deg": Float(default=0.0),
        "deadband_deg": Float(default=0.0),
        "stale_after": Float(default=0.75),
        "require_calibration": Bool(default=True),
        "require_leader_released": Bool(default=True),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool, "live": Bool, "armed": Bool, "commanded": Bool,
        "leader_pose": Dict, "follower_pose": Dict, "target": Dict,
        "sample_stream": Dict,
        "clamped": List, "joint_count": Int, "dashboard": Image,
        "summary": Dict, "report": Text,
    },
)
def ros2_leader_follower(ctx: dict) -> dict:
    return leader_follower_runtime.run_leader_follower(ctx)


@node(
    name="ROS2JointSubscribe",
    live=True,
    category=_CATEGORY,
    description="Maintain a managed ROS 2 JointState subscription and expose its fresh latest-value stream.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop", "check"], default="start"),
        "run_id": Text(default="joint_subscription"),
        "robot": Dict,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "config_topic": Text(default="/joint_config"),
        "stale_after": Float(default=0.75),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool, "live": Bool, "subscription": Dict,
        "sample_stream": Dict, "pose": Dict, "age": Float, "report": Text,
    },
)
def ros2_joint_subscribe(ctx: dict) -> dict:
    return leader_subscription_runtime.run_leader_subscription({
        **ctx,
        "leader_robot": ctx.get("robot"),
    })


@node(
    name="ROS2JointPublish",
    live=True,
    category=_CATEGORY,
    description="Publish safety-gated ROS 2 joint commands from a managed joint subscription.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop", "check"], default="start"),
        "run_id": Text(default="joint_publisher"),
        "control_topic": Text(default=""),
        "subscription": Dict,
        "robot": Dict,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "joint_map": Dict, "scale": Dict, "offset_deg": Dict,
        "tracking_mode": Enum(["bounded", "direct"], default="direct"),
        "loop_hz": Float(default=60.0),
        "max_step_deg": Float(default=0.0),
        "deadband_deg": Float(default=0.0),
        "stale_after": Float(default=0.75),
        "require_calibration": Bool(default=True),
        "require_leader_released": Bool(default=True),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool, "live": Bool, "armed": Bool, "published": Bool,
        "source_pose": Dict, "current_pose": Dict, "command": Dict,
        "message_stream": Dict, "clamped": List, "joint_count": Int,
        "dashboard": Image, "summary": Dict, "report": Text,
    },
)
def ros2_joint_publish(ctx: dict) -> dict:
    result = leader_follower_runtime.run_leader_follower({
        **ctx,
        "leader_subscription": ctx.get("subscription"),
        "follower_robot": ctx.get("robot"),
    })
    return {
        **result,
        "published": bool(result.get("commanded")),
        "source_pose": dict(result.get("leader_pose") or {}),
        "current_pose": dict(result.get("follower_pose") or {}),
        "command": dict(result.get("target") or {}),
        "message_stream": dict(result.get("sample_stream") or {}),
    }


@node(
    name="ROS2LeaderJointSubscriber",
    live=True,
    hidden=True,
    category=_CATEGORY,
    description="Compatibility alias for ROS2JointSubscribe.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop", "check"], default="start"),
        "run_id": Text(default="leader_joint_subscription"),
        "leader_robot": Dict,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/leader/joint_states"),
        "config_topic": Text(default="/leader/joint_config"),
        "stale_after": Float(default=0.75),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool, "live": Bool, "subscription": Dict,
        "sample_stream": Dict, "pose": Dict, "age": Float, "report": Text,
    },
)
def ros2_leader_joint_subscriber(ctx: dict) -> dict:
    return leader_subscription_runtime.run_leader_subscription(ctx)


@node(
    name="ROS2FollowerJointPublisher",
    live=True,
    hidden=True,
    category=_CATEGORY,
    description="Compatibility alias for ROS2JointPublish.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop", "check"], default="start"),
        "run_id": Text(default="follower_joint_publisher"),
        "control_topic": Text(default=""),
        "leader_subscription": Dict,
        "follower_robot": Dict,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "joint_map": Dict, "scale": Dict, "offset_deg": Dict,
        "tracking_mode": Enum(["bounded", "direct"], default="direct"),
        "loop_hz": Float(default=60.0),
        "max_step_deg": Float(default=0.0),
        "deadband_deg": Float(default=0.0),
        "stale_after": Float(default=0.75),
        "require_calibration": Bool(default=True),
        "require_leader_released": Bool(default=True),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool, "live": Bool, "armed": Bool, "commanded": Bool,
        "leader_pose": Dict, "follower_pose": Dict, "target": Dict,
        "sample_stream": Dict, "clamped": List, "joint_count": Int,
        "dashboard": Image, "summary": Dict, "report": Text,
    },
)
def ros2_follower_joint_publisher(ctx: dict) -> dict:
    return leader_follower_runtime.run_leader_follower(ctx)
