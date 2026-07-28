# blacknode-skills

This repository is the task-level skills layer. Each component composes stable
robot, perception, controller, and agent capabilities. Skills must not import
vendor SDKs or bind directly to device paths. Components begin disabled until
their executable nodes and dependency declarations are added.

Planned components: `pick-place`, `follow`, `delivery`, `docking`, and
`inspection`.

## ROS 2 leader and follower deployment

The template gallery presents two independently deployable ROS 2 workflows:

- **ROS 2 Leader Deploy** starts the selected robot in read-only mode
  and publishes its live joint state on `/leader/joint_states`.
- **ROS 2 Follower Deploy** starts the selected follower robot, subscribes to
  `/leader/joint_states`, and applies fresh calibrated poses after the
  **Arm follower** control is explicitly enabled.

The leader publisher preserves the selected robot's torque state. The follower
starts its robot driver, persistent subscription, and bounded command service
as one deployment. Motion begins only after explicit authorization and stops
when the source becomes stale.

Start the publisher first and the subscriber second. Linux deployments use
native `rclpy`. Systems on separate computers use the same ROS domain and a
network that permits DDS discovery. `ROS2JointSubscribe` owns the persistent
subscription and freshness state. The follower's pose-application node owns
the safety-gated local command stream. `ROS2LeaderJointSubscriber` and
`ROS2FollowerJointPublisher` remain hidden compatibility aliases for saved
workflows.
