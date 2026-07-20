"""ROS 2 leader/follower teleoperation node.

Delegates its run lifecycle to
:mod:`blacknode.pkg.blacknode_skills.follow_person.leader_follower_runtime`
so Stop All and status reporting can reach it without importing this
node-decorated module.
"""
from __future__ import annotations

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

from blacknode.pkg.blacknode_skills.follow_person import leader_follower_runtime

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
        "leader_robot": Dict,
        "follower_robot": Dict,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
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
