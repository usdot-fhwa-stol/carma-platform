# SDSM to detection list Node

This Node converts incoming `carma_v2x_msgs/SensorDataSharingMessage.msg` messages into
`carma_cooperative_perception/DetectionList.msg` messages. Similar to the Detection from basic safety message (BSM)
Node, this node performs some coordinate changes. The Node projects the SDSM's reference position from the WGS-84 datum
to the local UTM zone’s coordinate frame. It also converts the detected objects' positions, represented as Euclidean
offsets, to the same local UTM zone’s coordinate frame.

## Subscriptions

| Topic          | Message Type                                  | Description    |
| -------------- | --------------------------------------------- | -------------- |
| `~/input/sdsm` | `carma_v2x_msgs/SensorDataSharingMessage.msg` | Incoming SDSMs |

## Publishers

| Topic                 | Message Type                                                | Frequency           | Description              |
| --------------------- | ----------------------------------------------------------- | ------------------- | ------------------------ |
| `~/output/detections` | `carma_cooperative_perception_interfaces/DetectionList.msg` | Subscription-driven | Outgoing detection lists |

## Parameters

| Topic                    | Data Type | Default Value | Required | Read Only | Description                                                                    |
| ------------------------ | --------- | ------------- | -------- | --------- | ------------------------------------------------------------------------------ |
| `~/unknown_motion_model` | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `UNKNOWN` sematic class |
| `~/vehicle_motion_model` | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `VEHICLE` sematic class |
| `~/vru_motion_model`     | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `VRU` sematic class     |
| `~/animal_motion_model`  | `string`  | `''`          | Yes      | Yes       | Motion model assigned to detected object types with an `ANIMAL` sematic class  |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
