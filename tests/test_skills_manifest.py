import json
from pathlib import Path
from unittest.mock import patch

from blacknode.packages import load_package


def test_skills_layer_catalog_loads_with_components_disabled():
    with patch("blacknode.packages._read_component_overrides", return_value=({}, "")):
        info = load_package(Path(__file__).resolve().parents[1])
    assert info.ok
    assert info.layer == "skills"
    assert info.component_mode is True
    assert info.enabled_components == []
    assert set(info.components) == {"pick-place", "follow", "delivery", "docking", "inspection"}
    assert info.components["follow"]["aliases"] == ["follow-person"]


def test_follow_person_ros2_template_declares_every_adapter_it_uses():
    path = (
        Path(__file__).resolve().parents[1]
        / "components" / "follow" / "adapters" / "ros2" / "templates"
        / "so-arm101-leader-follower.json"
    )
    workflow = json.loads(path.read_text(encoding="utf-8"))
    # The owned adapter, plus the joint-control adapter that owns ROS2ManualMove.
    assert workflow["metadata"]["required_adapters"] == [
        "blacknode-skills/follow@ros2",
        "blacknode-controllers/joint-control@ros2",
    ]
    assert workflow["node_meta"]["subscribe"]["type"] == "ROS2JointSubscribe"
    assert workflow["node_meta"]["publish"]["type"] == "ROS2JointPublish"
    assert "follow" not in workflow["node_meta"]


def test_split_leader_follower_templates_are_one_robot_deployments():
    template_dir = (
        Path(__file__).resolve().parents[1]
        / "components" / "follow" / "adapters" / "ros2" / "templates"
    )
    leader = json.loads(
        (template_dir / "so-arm101-leader-deploy.json").read_text(encoding="utf-8")
    )
    follower = json.loads(
        (template_dir / "so-arm102-follower-deploy.json").read_text(encoding="utf-8")
    )

    assert leader["name"] == "SO-ARM101 Leader Deploy"
    assert follower["name"] == "SO-ARM102 Follower Deploy"
    assert leader["metadata"]["hidden"] is True
    assert follower["metadata"]["hidden"] is True
    assert sum(
        node["type"] == "Robot" for node in leader["node_meta"].values()
    ) == 1
    assert sum(
        node["type"] == "Robot" for node in follower["node_meta"].values()
    ) == 1
    assert leader["node_meta"]["leader_bridge"]["params"]["port"] == 9091
    assert leader["node_meta"]["leader_bridge"]["params"]["transport"] == "auto"
    assert leader["node_meta"]["release_leader"]["params"]["transport"] == "auto"
    assert leader["node_meta"]["release_leader"]["params"]["live_monitor"] is False
    assert leader["node_meta"]["leader_robot"]["params"]["read_only"] is True
    assert leader["node_meta"]["leader_bridge"]["params"]["expose_lan"] is False
    assert leader["node_meta"]["share_on_lan"]["params"] == {
        "value": False,
        "label": "Separate computers: expose leader on LAN",
    }
    assert "leader_robot_index" not in leader["node_meta"]
    assert leader["node_meta"]["leader_robot"]["params"]["selection"] == 0
    assert not any(edge["to_port"] == "selection" for edge in leader["edges"])
    assert follower["node_meta"]["subscribe"]["params"]["port"] == 9091
    assert follower["node_meta"]["follower_bridge"]["params"]["transport"] == "auto"
    assert follower["node_meta"]["subscribe"]["params"]["transport"] == "auto"
    assert follower["node_meta"]["publish"]["params"]["transport"] == "auto"
    assert follower["node_meta"]["publish"]["params"]["armed"] is False
    assert "follow" not in follower["node_meta"]
    assert "follower_robot_index" not in follower["node_meta"]
    assert follower["node_meta"]["follower_robot"]["params"]["selection"] == 0
    assert not any(edge["to_port"] == "selection" for edge in follower["edges"])


def test_visible_ros2_joint_pair_is_transport_only():
    template_dir = (
        Path(__file__).resolve().parents[1]
        / "components" / "follow" / "adapters" / "ros2" / "templates"
    )
    leader = json.loads(
        (template_dir / "ros2-joint-leader-deploy.json").read_text(
            encoding="utf-8"
        )
    )
    follower = json.loads(
        (template_dir / "ros2-joint-follower-deploy.json").read_text(
            encoding="utf-8"
        )
    )
    combined = json.loads(
        (template_dir / "so-arm101-leader-follower.json").read_text(
            encoding="utf-8"
        )
    )

    assert combined["metadata"]["hidden"] is True
    assert leader["metadata"].get("hidden") is not True
    assert follower["metadata"].get("hidden") is not True
    visible_deployments = {
        path.name
        for path in template_dir.glob("*deploy.json")
        if not json.loads(path.read_text(encoding="utf-8"))
        .get("metadata", {})
        .get("hidden", False)
    }
    assert visible_deployments == {
        "ros2-joint-leader-deploy.json",
        "ros2-joint-follower-deploy.json",
    }
    assert leader["metadata"]["deployment_pair"] == {
        "id": "ros2_joint_transport",
        "role": "publisher",
        "transport": "native",
        "state_topic": "/leader/joint_states",
        "message_type": "sensor_msgs/msg/JointState",
    }
    assert follower["metadata"]["deployment_pair"] == {
        "id": "ros2_joint_transport",
        "role": "subscriber",
        "transport": "native",
        "state_topic": "/leader/joint_states",
        "message_type": "sensor_msgs/msg/JointState",
    }
    assert leader["name"] == "ROS 2 Joint Leader Publish"
    assert follower["name"] == "ROS 2 Joint Follower Subscribe"
    assert set(leader["node_meta"]) == {"publish", "out"}
    assert set(follower["node_meta"]) == {"subscribe", "out"}
    assert leader["node_meta"]["publish"]["type"] == "Robot"
    assert leader["node_meta"]["publish"]["params"]["profile_id"] == "auto"
    assert leader["node_meta"]["publish"]["params"]["read_only"] is True
    assert leader["node_meta"]["publish"]["params"]["label"] == (
        "ROS 2 Publish: /leader/joint_states"
    )
    assert leader["node_meta"]["publish"]["params"]["state_topic"] == (
        "/leader/joint_states"
    )
    assert all(
        node["type"] not in {"ROS2ManualMove", "ROS2JointPublish"}
        for node in leader["node_meta"].values()
    )
    assert follower["node_meta"]["subscribe"]["type"] == "ROS2JointSubscribe"
    assert follower["node_meta"]["subscribe"]["params"]["label"] == (
        "ROS 2 Subscribe: /leader/joint_states"
    )
    assert follower["node_meta"]["subscribe"]["params"]["state_topic"] == (
        "/leader/joint_states"
    )
    assert all(
        node["type"] not in {"Robot", "Bool", "ROS2JointPublish", "ROS2ManualMove"}
        for node in follower["node_meta"].values()
    )
    assert any(
        edge["from"] == "subscribe"
        and edge["from_port"] == "pose"
        and edge["to"] == "out"
        and edge["to_port"] == "value"
        for edge in follower["edges"]
    )
