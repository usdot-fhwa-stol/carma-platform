# Host vehicle filter Node

This Node removes detections that are likely to associate with the host vehicle. For each detection in the received
detection list, the Node will check the distance between it and the host vehicle's current pose. Any detections that
fall below the specified distance threshold will be pruned from the incoming detection list. The Node will then
republish the (possibly pruned) list.

## Subscriptions

| Topic                       | Message Type                                                                      | Description                     |
| --------------------------- | --------------------------------------------------------------------------------- | ------------------------------- |
| `~/input/detection_list`    | [`carma_cooperative_perception_interfaces/DetectionList.msg`][detection_list_msg] | Incoming detections             |
| `~/input/host_vehicle_pose` | [`geometry_msgs/PoseStamped.msg`][pose_stamped_msg]                               | The host vehicle's current pose |

[pose_stamped_msg]: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html

## Publishers

| Topic                     | Message Type                                                                      | Frequency           | Description                                                                            |
| ------------------------- | --------------------------------------------------------------------------------- | ------------------- | -------------------------------------------------------------------------------------- |
| `~/output/detection_list` | [`carma_cooperative_perception_interfaces/DetectionList.msg`][detection_list_msg] | Subscription-driven | Incoming detections excluding any detections likely associating with the host vehicle. |

## Parameters

| Topic                  | Data Type | Default Value | Required | Read Only | Description                                                                       |
| ---------------------- | --------- | ------------- | -------- | --------- | --------------------------------------------------------------------------------- |
| `~/distance_threshold_meters` | `float`   | `0.0`         | No       | No        | Distance below which a detection will be considered to represent the host vehicle |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.

[detection_list_msg]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_cooperative_perception_interfaces/msg/DetectionList.msg
