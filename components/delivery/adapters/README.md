# Adapters

Transport adapters for the `delivery` component of `blacknode-skills`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.delivery.adapters.ros2]
    description = "ROS 2 adapter for delivery."
    default = false
    capabilities = ["adapter.delivery.ros2"]
    nodes = ["components/delivery/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
