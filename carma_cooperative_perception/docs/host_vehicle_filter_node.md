# Host vehicle filter Node

This Node removes detections that are likely to associate with the host vehicle. For each detection in the received
detection list, the Node will check the distance between it and the host vehicle's current pose. Any detections that
fall below the specified distance threshold will be pruned from the incoming detection list. The Node will then
republish the (possibly pruned) list.

## Subscriptions

| Topic                       | Message Type                                                | Description                     |
| --------------------------- | ----------------------------------------------------------- | ------------------------------- |
| `~/input/detections`        | `carma_cooperative_perception_interfaces/DetectionList.msg` | Incoming detections             |
| `~/input/host_vehicle_pose` | `??`                                                        | The host vehicle's current pose |

## Publishers

| Topic                 | Message Type                                                | Frequency           | Description                                                                            |
| --------------------- | ----------------------------------------------------------- | ------------------- | -------------------------------------------------------------------------------------- |
| `~/output/detections` | `carma_cooperative_perception_interfaces/DetectionList.msg` | Subscription-driven | Incoming detections excluding any detections likely associating with the host vehicle. |

## Parameters

| Topic                  | Data Type | Default Value | Required | Read Only | Description                                                                       |
| ---------------------- | --------- | ------------- | -------- | --------- | --------------------------------------------------------------------------------- |
| `~/distance_threshold` | `float`   | ``            | No       | No        | Distance below which a detection will be considered to represent the host vehicle |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
