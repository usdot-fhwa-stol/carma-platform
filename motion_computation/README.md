# CARMA motion computation package

The `motion_computation` package contains a node that subscribes to ROS topics
containing position, velocity, and other relevant information for external
objects. Using this received data, this node predicts the future location of
each of these external objects, and publishes those predictions for other nodes
within the CARMA system to utilize.

## Package Nodes

- Motion computation Node: [`motion_computation_node`][motion_computation_node_docs]

[motion_computation_node_docs]: docs/motion_computation_node.md

## Package Launch files

- Motion computation Launch: [`motion_computation_launch.py`][motion_computation_launch_docs]

[motion_computation_launch_docs]: docs/motion_computation_launch.md
