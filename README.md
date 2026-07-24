# blacknode-skills

This repository is the task-level skills layer. Each component composes stable
robot, perception, controller, and agent capabilities. Skills must not import
vendor SDKs or bind directly to device paths. Components begin disabled until
their executable nodes and dependency declarations are added.

Planned components: `pick-place`, `follow-person`, `delivery`, `docking`, and
`inspection`.

## Split leader and follower deployment

The follow-person ROS 2 adapter ships two one-robot deployment templates:

- **SO-ARM101 Leader Deploy** starts the selected leader robot, releases its
  torque for hand guidance, and publishes its joint stream on ROS 2.
- **SO-ARM102 Follower Deploy** starts the selected follower robot, reads the
  leader stream, and keeps follower motion disarmed until its `Armed` value is
  explicitly enabled.

Deploy the leader first and the follower second. Linux deployments select
native `rclpy`; two robots on one computer share its ROS 2 graph directly.
For separate Linux computers, use the same ROS domain and a network where DDS
discovery is permitted. Windows local runs select rosbridge automatically and
retain the host, port, and LAN-exposure controls as their compatibility path.

`SO-ARM102 Follower Deploy` is a deployment-role template name. Blacknode
currently ships the `so_arm101` mechanical profile as its default. Select the
saved follower profile and its hardware-bound calibration in the deployment
panel; the template does not invent SO-ARM102 joint geometry.
