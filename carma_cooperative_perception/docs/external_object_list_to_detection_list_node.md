# External object list to detection list Node

This Node converts incoming `carma_perception_msgs/ExternalObjectList.msg` messages into
`carma_cooperative_perception_interfaces/DetectionList.msg` messages. It converts each external object’s position data
from the host vehicle's map frame to its UTM zone’s coordinate frame.

## Subscriptions

| Topic                      | Message Type                                   | Description                                                                                     |
| -------------------------- | ---------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `~/input/external_objects` | `carma_perception_msgs/ExternalObjectList.msg` | Incoming external objects perceived locally                                                     |
| `~/input/georeference`     | `std_msgs/String.msg`                          | PROJ string representing the projection from WGS-84 coordinates to the host vehicle's map frame |

## Publishers

| Topic                 | Message Type                                                | Frequency           | Description         |
| --------------------- | ----------------------------------------------------------- | ------------------- | ------------------- |
| `~/output/detections` | `carma_cooperative_perception_interfaces/DetectionList.msg` | Subscription-driven | Outgoing detections |

## Parameters

| Topic                          | Data Type | Default Value                                                                                 | Required | Read Only | Description                                                                          |
| ------------------------------ | --------- | --------------------------------------------------------------------------------------------- | -------- | --------- | ------------------------------------------------------------------------------------ |
| `~/unknown_motion_model`       | `int64`   | `3` (defined by `carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CV`)   | No      | No       | Motion model assigned to detected object types with an `UNKNOWN` semantic class       |
| `~/small_vehicle_motion_model` | `int64`   | `1` (defined by `carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRV`) | No      | No       | Motion model assigned to detected object types with an `SMALL_VEHICLE` semantic class |
| `~/large_vehicle_motion_model` | `int64`   | `1` (defined by `carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRV`) | No      | No       | Motion model assigned to detected object types with an `LARGE_VEHICLE` semantic class |
| `~/motorcycle_motion_model`    | `int64`   | `2` (defined by `carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRA`) | No      | No       | Motion model assigned to detected object types with an `MOTORCYCLE` semantic class    |
| `~/pedestrian_motion_model`    | `int64`   | `3` (defined by `carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CV`)   | No      | No       | Motion model assigned to detected object types with an `PEDESTRIAN` semantic class    |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
