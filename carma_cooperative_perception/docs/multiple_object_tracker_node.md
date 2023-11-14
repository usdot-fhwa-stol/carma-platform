# Multiple object tracker

This is the main Node in the package; it does the actual object tracking. It wraps the `multiple_object_tracking`
library in a ROS 2 interface and composes the individual tracking steps into a cohesive pipeline. Check out the
library's documentation for details on the specific pipeline steps. For our purposes, we need to be concerned only with
the node's inputs and outputs. It receives incoming `carma_cooperative_perception_interfaces/DetectionList.msg` messages
from various sources (CARMA Platform and connected remote actors), and sends out
`carma_cooperative_perception_interfaces/TrackList.msg` messages. This Node fuses the data from the incoming
`DetectionList.msg` messages to form the outgoing `TrackList.msg` messages' data.

## Node behavior

The `multiple_object_tracker_node` Node executes a multiple object tracking pipeline at a specific frequency, which the
`execution_frequency_hz` parameter defines. Between executions, the Node listens for incoming `DetectionList.msg`
messages on the `~/input/detections` topic, queueing received detections for processsing. The tracking pipeline
executes the following steps on each iteration:

1. temporal alignment
2. detection-to-track scoring
3. detection-to-track association
4. unassociated detection clustering
5. track maintenance
6. detection-to-track fusion

The tracker Node outputs a list of confirmed tracks after executing the pipeline.

## Subscriptions

| Topic                | Message Type                                                                           | Description         |
| -------------------- | -------------------------------------------------------------------------------------- | ------------------- |
| `~/input/detections` | [`carma_cooperative_perception_interfaces/DetectionList.msg`][detection_list_msg_link] | Incoming detections |

[detection_list_msg_link]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_cooperative_perception_interfaces/msg/DetectionList.msg

## Publishers

| Topic             | Message Type                                                                   | Frequency         | Description                                                                                   |
| ----------------- | ------------------------------------------------------------------------------ | ----------------- | --------------------------------------------------------------------------------------------- |
| `~/output/tracks` | [`carma_cooperative_perception_interfaces/TrackList.msg`][track_list_msg_link] | Parameter-defined | Tracked objects from the pipeline. **Note:** The track list contains only _confirmed_ tracks. |

[track_list_msg_link]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_cooperative_perception_interfaces/msg/TrackList.msg

## Parameters

| Topic                      | Data Type | Default Value | Required | Read Only | Description                                                  |
| -------------------------- | --------- | ------------- | -------- | --------- | ------------------------------------------------------------ |
| `~/execution_frequency_hz` | `float`   | `2.0`         | No       | No        | Tracking execution pipeline's execution frequency (in Hertz) |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
