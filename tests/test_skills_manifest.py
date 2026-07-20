import json
from pathlib import Path

from blacknode.packages import load_package


def test_skills_layer_catalog_loads_with_components_disabled():
    info = load_package(Path(__file__).resolve().parents[1])
    assert info.ok
    assert info.layer == "skills"
    assert info.component_mode is True
    assert info.enabled_components == []
    assert set(info.components) == {"pick-place", "follow-person", "delivery", "docking", "inspection"}


def test_follow_person_ros2_template_declares_owned_adapter():
    path = (
        Path(__file__).resolve().parents[1]
        / "components" / "follow-person" / "adapters" / "ros2" / "templates"
        / "so-arm101-leader-follower.json"
    )
    workflow = json.loads(path.read_text(encoding="utf-8"))
    assert workflow["metadata"]["required_adapters"] == ["blacknode-skills/follow-person@ros2"]
    assert workflow["node_meta"]["follow"]["type"] == "ROS2LeaderFollower"
