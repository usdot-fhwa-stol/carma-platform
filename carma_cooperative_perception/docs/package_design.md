# CARMA cooperative perception - ROS 2 package design

This package provides the necessary ROS 2 Nodes and Launch files to support cooperative perception within CARMA
Platform. Message conversion Nodes allow the stack to ingest heterogeneous data sources by converting unique messages
types to a `carma_cooperative_perception/DetectionList.msg` message, which gets passed into a multiple-object tracking
pipeline. On the pipeline's output, more message conversion Nodes adapt the data to a format that is compatible with
the rest of CARMA Platform.

## ROS 2 Nodes

This package's ROS 2 Node composition is similar to a micro-services architecture. Let's call it a micro-node
architecture. Each Node in the package is responsible for a single task, either adapting a message type or tracking
objects. This architecture promotes testability and helps us decouple each concern's implementation. For example, the
main pipeline, the `multiple_object_tracker_node` Node, should not care about how message types get converted. The
subsystem's Nodes are managed (i.e., [ROS 2 Lifecycle Nodes][ros2_lifecycle_nodes_link]) and implemented as
[ROS 2 Components][ros2_components_link].

In its full capacity, the cooperative perception stack comprises two concurrent dataflows. One flow is responsible for
tracking and fusing object data from incoming basic safety message (BSMs), sensor data sharing messages (SDSMs), and
locally-perceived objects. This flow outputs only to the host; it does not share the results with the host’s neighbors.
The second dataflow fuses data coming from only locally-perceived objects, and the host shares the outputs from this
flow with its neighboring actors (e.g., other connected vehicles or infrastructure).

### Package Nodes

- Basic safety message (BSM) to detection list Node[^1]: [`bsm_to_detection_list_node`][bsm_to_detection_list_node_docs]
- External object list to detection list Node: [`external_object_list_to_detection_list_node`][external_object_list_to_detection_list_node_docs]
- Sensor data sharing message (SDSM) to detection list Node: [`sdsm_to_detection_list_node`][sdsm_to_detection_list_node_docs]
- Multiple object tracker Node: [`multiple_object_tracker_node`][multiple_object_tracker_node_docs]
- Track list to external object list: [`external_object_list_from_track_node`][track_list_to_external_object_list_node_docs]
- External object list to sensor data sharing message (SDSM) Node: [`sdsm_from_external_object_list_node`][external_object_list_to_sdsm_node_docs]
- Basic safety message (BSM) filter Node[^1]: [`bsm_filter_node`][bsm_filter_node_docs]

[^1]: These Nodes are currently unimplemented.

[ros2_lifecycle_nodes_link]: https://design.ros2.org/articles/node_lifecycle.html
[ros2_components_link]: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Composition.html
[bsm_to_detection_list_node_docs]: bsm_to_detection_list_node.md
[external_object_list_to_detection_list_node_docs]: external_object_list_to_detection_list_node.md
[sdsm_to_detection_list_node_docs]: sdsm_to_detection_list_node.md
[multiple_object_tracker_node_docs]: multiple_object_tracker_node.md
[track_list_to_external_object_list_node_docs]: track_list_to_external_object_list_node.md
[external_object_list_to_sdsm_node_docs]: external_object_list_to_sdsm_node.md
[bsm_filter_node_docs]: bsm_filter_node.md

## ROS 2 launch configurations

The `cooperative_perception.launch.py` Launch file allows user-integrators to spin up the cooperative perception stack
without worrying about the node composition. From this perspective, we can view the multiple-object perception pipeline
as a monolithic subsystem. This subsystem's external inputs are BSMs, SDSMs, and locally-perceived objects. It's
external outputs are SDSMs and external object lists.

This launch file brings up the fully-featured cooperative perception stack. It spins up the Nodes within a CARMA
component container (same as the ROS 2 component container but with logging).

### Launch arguments

This Launch file does not provide launch arguments.
