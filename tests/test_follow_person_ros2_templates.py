"""Structural checks on the vision-driven follow-person@ros2 mission templates.

These moved out of blacknode-vision (now blacknode-perception) along with the follow-node types they
reference (see the embodied-robotics-roadmap Stage D item on extracting
follow-target/leader-follower behavior into this adapter).
"""
from __future__ import annotations

import json
from pathlib import Path

TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "components" / "follow-person" / "adapters" / "ros2" / "templates"


def test_cube_template_uses_live_cv2_stream_and_qwen3():
    path = TEMPLATE_DIR / "vision-cv2-cube-local-reasoning.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    assert workflow["node_meta"]["stream"]["type"] == "Camera"
    assert workflow["node_meta"]["stream"]["params"]["selection"] == 0
    assert "backend" not in workflow["node_meta"]["stream"]["params"]
    assert "camera_run" not in workflow["node_meta"]
    assert workflow["node_meta"]["cv2_stream"]["type"] == "CV2ColorObjectStream"
    assert workflow["node_meta"]["target_prompt"]["type"] == "Text"
    assert "green cube" not in workflow["node_meta"]["target_prompt"]["params"]["value"].lower()
    assert "target_hint" not in workflow["node_meta"]
    assert "python_export" not in workflow["node_meta"]
    assert workflow["node_meta"]["cv2_stream"]["params"]["object_color"] == "#22c55e"
    assert workflow["node_meta"]["cv2_stream"]["params"]["use_reasoning_color"] is True
    assert "tracking_mode" not in workflow["node_meta"]["cv2_stream"]["params"]
    assert "fallback_color" not in workflow["node_meta"]["cv2_stream"]["params"]
    assert "lower_hsv" not in workflow["node_meta"]["cv2_stream"]["params"]
    assert workflow["node_meta"]["live_reason"]["type"] == "ReasoningStream"
    assert workflow["node_meta"]["live_reason"]["params"]["model"] == "qwen3-vl:4b"
    assert workflow["node_meta"]["live_reason"]["params"]["max_tokens"] == 4096
    assert workflow["node_meta"]["live_reason"]["params"]["interval_seconds"] == 3.0
    assert workflow["node_meta"]["live_reason"]["params"]["max_fps"] == 4.0
    assert "Describe what you see" in workflow["node_meta"]["live_reason"]["params"]["prompt"]
    assert "Do not rely on CV2 detections" in workflow["node_meta"]["live_reason"]["params"]["system"]
    edges = {
        (edge["from"], edge["from_port"], edge["to"], edge["to_port"])
        for edge in workflow["edges"]
    }
    assert ("stream", "snapshot_url", "cv2_stream", "source_url") in edges
    assert ("target_prompt", "value", "live_reason", "prompt") in edges
    assert ("live_reason", "state_url", "cv2_stream", "reasoning_state_url") in edges
    assert ("target_prompt", "value", "cv2_stream", "target") not in edges
    assert ("cv2_stream", "preview", "overlay_out", "image") in edges
    assert ("cv2_stream", "mask", "mask_out", "image") in edges
    assert ("stream", "snapshot_url", "live_reason", "image_url") in edges
    assert ("live_reason", "preview", "reason_dashboard_out", "image") in edges
    assert ("cv2_stream", "detection_url", "live_reason", "detection_url") not in edges
    assert workflow["node_meta"]["check"]["type"] == "ROS2Status"
    assert workflow["node_meta"]["robot"]["type"] == "Robot"
    assert workflow["node_meta"]["robot"]["params"]["profile_id"] == "so_arm101"
    assert workflow["node_meta"]["robot"]["params"]["selection"] == 0
    assert sum(meta["type"] == "Robot" for meta in workflow["node_meta"].values()) == 1
    assert workflow["node_meta"]["joint_state"]["type"] == "ROS2JointState"
    assert workflow["node_meta"]["follow_cube"]["type"] == "ROS2ContinuousFollowDetectionJoint"
    assert workflow["node_meta"]["follow_cube"]["params"]["action"] == "start"
    assert workflow["node_meta"]["follow_cube"]["params"]["loop_hz"] == 2.0
    assert workflow["node_meta"]["follow_cube"]["params"]["joint"] == "shoulder_pan"
    assert workflow["node_meta"]["follow_cube"]["params"]["armed"] is False
    assert workflow["node_meta"]["follow_cube"]["params"]["frame_width"] == 640
    assert workflow["node_meta"]["follow_cube"]["params"]["target_x"] == 0.4
    assert workflow["node_meta"]["follow_cube"]["params"]["deadband"] == 0.12
    assert workflow["node_meta"]["follow_cube"]["params"]["gain"] == 10.0
    assert workflow["node_meta"]["follow_cube"]["params"]["max_step"] == 2.0
    assert workflow["node_meta"]["cv2_stream"]["params"]["show_follow_guides"] is True
    assert workflow["node_meta"]["cv2_stream"]["params"]["follow_target_x"] == 0.4
    assert workflow["node_meta"]["cv2_stream"]["params"]["follow_deadband"] == 0.12
    assert "shoulder_pan_index" not in workflow["node_meta"]
    assert ("joint_state", "names", "shoulder_pan_index", "items") not in edges
    assert ("shoulder_pan_index", "value", "follow_cube", "joint") not in edges


def test_cube_ros2_template_keeps_ros_camera_and_generic_robot_transport():
    path = TEMPLATE_DIR / "vision-cv2-cube-ros2-native-reasoning.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    node_types = {node_id: meta["type"] for node_id, meta in workflow["node_meta"].items()}
    edges = {
        (edge["from"], edge["from_port"], edge["to"], edge["to_port"])
        for edge in workflow["edges"]
    }

    assert node_types["camera_run"] == "ROS2Run"
    assert node_types["stream"] == "ROS2ImageStream"
    assert node_types["follow_cube"] == "ROS2FollowDetectionJoint"
    assert node_types["robot"] == "Robot"
    assert workflow["node_meta"]["robot"]["params"]["profile_id"] == "so_arm101"
    assert workflow["node_meta"]["robot"]["params"]["selection"] == 0
    assert sum(node_type == "Robot" for node_type in node_types.values()) == 1
    assert not any("Native" in node_type or "Rosbridge" in node_type for node_type in node_types.values())
    assert "CV2CameraStream" not in node_types.values()
    assert workflow["node_meta"]["camera_run"]["params"]["package"] == "blacknode_usb_camera"
    assert workflow["node_meta"]["stream"]["params"]["topic"] == "/camera/image_raw"
    assert ("check", "report", "camera_run", "trigger") in edges
    assert ("camera_run", "report", "stream", "trigger") in edges
    assert ("stream", "snapshot_url", "cv2_stream", "source_url") in edges


def test_cube_continuous_template_uses_generic_setup_nodes():
    path = TEMPLATE_DIR / "vision-cv2-cube-rosbridge-reasoning.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    node_types = {
        node_id: meta["type"]
        for node_id, meta in workflow["node_meta"].items()
    }
    package_names = {
        package["name"]
        for package in workflow["metadata"]["required_packages"]
    }
    edges = {
        (edge["from"], edge["from_port"], edge["to"], edge["to_port"])
        for edge in workflow["edges"]
    }

    assert {"blacknode-perception", "blacknode-ros2", "blacknode-skills", "blacknode-robot", "blacknode-cuda"} <= package_names
    assert not any("Native" in node_type or "Rosbridge" in node_type for node_type in node_types.values())
    assert node_types["check"] == "ROS2Status"
    assert node_types["stream"] == "Camera"
    assert "camera_run" not in node_types
    assert node_types["robot"] == "Robot"
    assert workflow["node_meta"]["robot"]["params"]["profile_id"] == "so_arm101"
    assert workflow["node_meta"]["robot"]["params"]["selection"] == 0
    assert sum(node_type == "Robot" for node_type in node_types.values()) == 1
    assert node_types["joint_state"] == "ROS2JointState"
    assert node_types["follow_cube"] == "ROS2ContinuousFollowDetectionJoint"
    assert workflow["node_meta"]["follow_cube"]["params"]["action"] == "start"
    assert workflow["node_meta"]["follow_cube"]["params"]["loop_hz"] == 10.0
    assert workflow["node_meta"]["follow_cube"]["params"]["armed"] is False
    assert workflow["node_meta"]["follow_cube"]["params"]["host"] == "127.0.0.1"
    assert workflow["node_meta"]["follow_cube"]["params"]["port"] == 9090
    assert workflow["node_meta"]["follow_cube"]["params"]["frame_width"] == 640
    assert workflow["node_meta"]["follow_cube"]["params"]["target_x"] == 0.4
    assert workflow["node_meta"]["follow_cube"]["params"]["deadband"] == 0.03
    assert workflow["node_meta"]["follow_cube"]["params"]["gain"] == 4.0
    assert workflow["node_meta"]["follow_cube"]["params"]["max_step"] == 0.75
    assert workflow["node_meta"]["cv2_stream"]["params"]["show_follow_guides"] is True
    assert workflow["node_meta"]["cv2_stream"]["params"]["follow_target_x"] == 0.4
    assert workflow["node_meta"]["cv2_stream"]["params"]["follow_deadband"] == 0.03
    assert ("stream", "snapshot_url", "cv2_stream", "source_url") in edges
    assert ("cv2_stream", "detection", "follow_cube", "detection") in edges
    assert ("cv2_stream", "detection_stream", "follow_cube", "detection_stream") in edges
    assert ("cv2_stream", "detection_url", "follow_cube", "detection_url") not in edges
    assert ("check", "report", "robot", "trigger") in edges
    assert not any(edge[0] == "preset" or edge[2] == "preset" for edge in edges)
    assert ("robot", "report", "joint_state", "trigger") in edges
    assert ("robot", "report", "follow_cube", "trigger") in edges
    assert ("robot", "robot", "follow_cube", "robot") in edges
    assert "shoulder_pan_index" not in node_types


def test_templates_declare_required_adapter():
    for name in [
        "vision-cv2-cube-local-reasoning.json",
        "vision-cv2-cube-ros2-native-reasoning.json",
        "vision-cv2-cube-rosbridge-reasoning.json",
    ]:
        workflow = json.loads((TEMPLATE_DIR / name).read_text(encoding="utf-8"))
        assert workflow["metadata"]["required_adapters"] == ["blacknode-skills/follow-person@ros2"]
