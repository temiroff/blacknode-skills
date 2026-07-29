# blacknode-skills

This repository is the task-level skills layer. Each component composes stable
robot, perception, controller, and agent capabilities. Skills must not import
vendor SDKs or bind directly to device paths. Components begin disabled until
their executable nodes and dependency declarations are added.

Planned components: `pick-place`, `follow`, `delivery`, `docking`, and
`inspection`.

## ROS 2 leader and follower deployment

The template gallery presents two independently deployable ROS 2 workflows:

- **ROS 2 Leader Deploy** shows the complete `Leader Robot` →
  `ROS 2 Publish Joint State` path and publishes the robot's live joint state
  on `/leader/joint_states`.
- **ROS 2 Follower Deploy** starts the selected follower robot, subscribes to
  `/leader/joint_states` through `ROS 2 Subscribe Joint State`, and passes fresh
  calibrated poses into `Joint Controller` after **Arm follower** is explicitly
  enabled.

The leader publisher preserves the selected robot's torque state. The follower
starts its robot driver, persistent subscription, and bounded command service
as one deployment. Motion begins only after explicit authorization and stops
when the source becomes stale.

Start the publisher first and the subscriber second. Linux deployments use
native `rclpy`. Systems on separate computers use the same ROS domain and a
network that permits DDS discovery. `ROS2PublishJointState` exposes the
publisher owned by the leader's running Robot driver and keeps that deployment
alive while the driver publishes.
`ROS2SubscribeJointState` owns the persistent subscription and freshness state.
`ROS2JointController` owns the safety-gated follower command stream, matching
the publisher → subscriber → controller vocabulary used by simulation action
graphs. The former `ROS2JointStatePublish`, `ROS2JointSubscribe`,
`ROS2JointReplicate`, `ROS2JointPublish`, `ROS2LeaderJointSubscriber`, and
`ROS2FollowerJointPublisher` names remain hidden compatibility aliases for
saved workflows.
