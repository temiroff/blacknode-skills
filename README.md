# blacknode-skills

This repository is the task-level skills layer. Each component composes stable
robot, perception, controller, and agent capabilities. Skills must not import
vendor SDKs or bind directly to device paths. Components begin disabled until
their executable nodes and dependency declarations are added.

Planned components: `pick-place`, `follow`, `delivery`, `docking`, and
`inspection`.

## Split leader and follower deployment

The follow ROS 2 adapter ships visual-follow and leader/follower nodes, plus
two one-robot deployment templates:

- **SO-ARM101 Leader Deploy** starts the selected leader robot, releases its
  torque for hand guidance, and publishes its joint stream on ROS 2.
- **SO-ARM102 Follower Deploy** starts the selected follower robot, reads the
  leader stream, and keeps follower motion disarmed until its `Armed` value is
  explicitly enabled. Enabling `Armed` seeds and enables holding torque on the
  follower at its current pose before sending bounded joint targets. Disabling
  `Armed`, stopping the controller, or shutting down the deployment explicitly
  releases follower torque. The controller never enables leader torque.

Running follower deployments also listen on their declared
`/blacknode/leader_follower/<run_id>/control` topic. The authenticated Runtime
uses that fixed topic for the editor's explicit **Arm follower** and
**Disarm follower** actions. A deployment always starts from the graph's
disarmed default; restart never restores a previous live arm command.

Deploy the leader first and the follower second. Linux deployments select
native `rclpy`; two robots on one computer share its ROS 2 graph directly.
For separate Linux computers, use the same ROS domain and a network where DDS
discovery is permitted. Windows local runs select rosbridge automatically and
retain the host, port, and LAN-exposure controls as their compatibility path.

The native **ROS 2 Joint Follower Deploy** canvas exposes the transport as two
managed nodes. `ROS2LeaderJointSubscriber` owns the persistent
`/leader/joint_states` subscription and freshness state.
`ROS2FollowerJointPublisher` consumes that subscription and publishes
calibrated, limited `/follower/joint_commands` only while explicitly armed.
`ROS2TopicEcho` remains a bounded diagnostic reader and is not part of this
deployment path.

`SO-ARM102 Follower Deploy` is a deployment-role template name. Blacknode
currently ships the `so_arm101` mechanical profile as its default. Select the
saved follower profile and its hardware-bound calibration in the deployment
panel; the template does not invent SO-ARM102 joint geometry.
