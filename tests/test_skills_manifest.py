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
    assert set(info.components) == {"pick-place", "follow-person", "delivery", "docking", "inspection"}


def test_follow_person_ros2_template_declares_every_adapter_it_uses():
    path = (
        Path(__file__).resolve().parents[1]
        / "components" / "follow-person" / "adapters" / "ros2" / "templates"
        / "so-arm101-leader-follower.json"
    )
    workflow = json.loads(path.read_text(encoding="utf-8"))
    # The owned adapter, plus the joint-control adapter that owns ROS2ManualMove.
    assert workflow["metadata"]["required_adapters"] == [
        "blacknode-skills/follow-person@ros2",
        "blacknode-controllers/joint-control@ros2",
    ]
    assert workflow["node_meta"]["follow"]["type"] == "ROS2LeaderFollower"


def test_split_leader_follower_templates_are_one_robot_deployments():
    template_dir = (
        Path(__file__).resolve().parents[1]
        / "components" / "follow-person" / "adapters" / "ros2" / "templates"
    )
    leader = json.loads(
        (template_dir / "so-arm101-leader-deploy.json").read_text(encoding="utf-8")
    )
    follower = json.loads(
        (template_dir / "so-arm102-follower-deploy.json").read_text(encoding="utf-8")
    )

    assert leader["name"] == "SO-ARM101 Leader Deploy"
    assert follower["name"] == "SO-ARM102 Follower Deploy"
    assert sum(
        node["type"] == "Robot" for node in leader["node_meta"].values()
    ) == 1
    assert sum(
        node["type"] == "Robot" for node in follower["node_meta"].values()
    ) == 1
    assert leader["node_meta"]["leader_bridge"]["params"]["port"] == 9091
    assert leader["node_meta"]["leader_bridge"]["params"]["expose_lan"] is False
    assert leader["node_meta"]["share_on_lan"]["params"] == {
        "value": False,
        "label": "Separate computers: expose leader on LAN",
    }
    assert follower["node_meta"]["follow"]["params"]["leader_port"] == 9091
    assert follower["node_meta"]["follow"]["params"]["armed"] is False
