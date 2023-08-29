# External object list to detection list Node

This Node converts incoming `carma_perception_msgs/ExternalObjectList.msg` messages into
`carma_cooperative_perception_interfaces/DetectionList.msg` messages. It converts each external object’s position data
from the host vehicle's coordinate frame to the its UTM zone’s coordinate frame.

## Subscriptions

| Topic                      | Message Type                                   | Description                                 |
| -------------------------- | ---------------------------------------------- | ------------------------------------------- |
| `~/input/external_objects` | `carma_perception_msgs/ExternalObjectList.msg` | Incoming external objects perceived locally |

## Publishers

| Topic                 | Message Type                                                | Frequency           | Description         |
| --------------------- | ----------------------------------------------------------- | ------------------- | ------------------- |
| `~/output/detections` | `carma_cooperative_perception_interfaces/DetectionList.msg` | Subscription-driven | Outgoing detections |

## Parameters

| Topic                          | Data Type | Default Value | Required | Read Only | Description                                                                          |
| ------------------------------ | --------- | ------------- | -------- | --------- | ------------------------------------------------------------------------------------ |
| `~/unknown_motion_model`       | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `UNKNOWN` sematic class       |
| `~/small_vehicle_motion_model` | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `SMALL_VEHICLE` sematic class |
| `~/large_vehicle_motion_model` | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `LARGE_VEHICLE` sematic class |
| `~/motorcycle_motion_model`    | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `MOTORCYCLE` sematic class    |
| `~/pedestrian_motion_model`    | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `PEDESTRIAN` sematic class    |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
