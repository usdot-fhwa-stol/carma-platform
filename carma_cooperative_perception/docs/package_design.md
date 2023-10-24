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

- Sensor data sharing message (SDSM) to detection list Node: [`sdsm_to_detection_list_node`][sdsm_to_detection_list_node_docs]
- External object list to detection list Node:
  [`external_object_list_to_detection_list_node`][external_object_list_to_detection_list_node_docs]
- Track list to external object list Node: [`track_list_to_external_object_list_node`][track_list_to_external_object_list_node_docs]
- Multiple object tracker Node: [multiple_object_tracker_node_docs]

[ros2_lifecycle_nodes_link]: https://design.ros2.org/articles/node_lifecycle.html
[ros2_components_link]: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Composition.html
[sdsm_to_detection_list_node_docs]: sdsm_to_detection_list_node.md
[external_object_list_to_detection_list_node_docs]: external_object_list_to_detection_list_node.md
[track_list_to_external_object_list_node_docs]: track_list_to_external_object_list_node.md
[multiple_object_tracker_node_docs]: multiple_object_tracker_node.md
