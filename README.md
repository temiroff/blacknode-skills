# blacknode-skills

This repository is the task-level skills layer. Each component composes stable
robot, perception, controller, and agent capabilities. Skills must not import
vendor SDKs or bind directly to device paths. Components begin disabled until
their executable nodes and dependency declarations are added.

Planned components: `pick-place`, `follow`, `delivery`, `docking`, and
`inspection`.

## ROS 2 publish and subscribe deployment

The template gallery presents two small ROS 2 transport workflows:

- **ROS 2 Joint Leader Publish** starts the selected robot in read-only mode
  and publishes its live joint state on `/leader/joint_states`.
- **ROS 2 Joint Follower Subscribe** subscribes to
  `/leader/joint_states` and exposes the latest received pose as its output.

The publisher preserves the selected robot's torque state. The subscriber is
an observation workflow. Robot command and hand-guided teleoperation behavior
remain available through the hidden compatibility templates for saved
workflows.

Start the publisher first and the subscriber second. Linux deployments use
native `rclpy`. Systems on separate computers use the same ROS domain and a
network that permits DDS discovery. `ROS2JointSubscribe` owns the persistent
subscription and freshness state. `ROS2LeaderJointSubscriber` and
`ROS2FollowerJointPublisher` remain hidden compatibility aliases for saved
workflows.
