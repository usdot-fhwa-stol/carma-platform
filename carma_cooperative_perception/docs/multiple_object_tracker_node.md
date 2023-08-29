# Multiple object tracker

This is the main Node in the package; it does the actual object tracking. It wraps the `cooperative_perception_core` library into a ROS 2 interface and composes the individual tracking steps into a cohesive pipeline. Check out the library's documentation for details on the specific pipeline steps. For our purposes, we need to be concerned only with the node's inputs and outputs. It receives incoming `carma_cooperative_perception_interfaces/DetectionList.msg` messages from various sources (CARMA Platform and connected remote actors), and sends out `carma_cooperative_perception_interfaces/TrackList.msg` messages. This Node fuses the data from the incoming Detection.msg messages to form the outgoing `TrackList.msg` messages' data.

## Subscriptions

| Topic                | Message Type                                                | Description         |
| -------------------- | ----------------------------------------------------------- | ------------------- |
| `~/input/detections` | `carma_cooperative_perception_interfaces/DetectionList.msg` | Incoming detections |

## Publishers

| Topic             | Message Type                                            | Frequency         | Description                       |
| ----------------- | ------------------------------------------------------- | ----------------- | --------------------------------- |
| `~/output/tracks` | `carma_cooperative_perception_interfaces/TrackList.msg` | Parameter-defined | Tracked objects from the pipeline |

## Parameters

This node does provide parameters.

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
