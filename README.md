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
  torque for hand guidance, and exposes its joint stream through rosbridge on
  TCP `9091`.
- **SO-ARM102 Follower Deploy** starts the selected follower robot, reads the
  leader stream, and keeps follower motion disarmed until its `Armed` value is
  explicitly enabled.

Deploy the leader first and the follower second. For two robots attached to one
computer, leave the follower's `placement` at `same_device` and
`leader_host=127.0.0.1`. For separate computers, choose `separate_devices` and
set `leader_host` to the leader computer's LAN IP or hostname. On the leader
template, also enable `Separate computers: expose leader on LAN`. The follower
uses its local rosbridge on `9090` and the leader endpoint on `9091`. The LAN
toggle is off by default.

Port `9091` is an unauthenticated rosbridge WebSocket and is intended for a
trusted robot LAN. Permit TCP `9091` on the leader computer only for the
follower computer or trusted subnet. No DDS multicast between the two
computers is required for this split deployment.

`SO-ARM102 Follower Deploy` is a deployment-role template name. Blacknode
currently ships the `so_arm101` mechanical profile as its default. Select the
saved follower profile and its hardware-bound calibration in the deployment
panel; the template does not invent SO-ARM102 joint geometry.
