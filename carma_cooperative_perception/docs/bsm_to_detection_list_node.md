# BSM to detection list Node

> [!IMPORTANT]\
> This Node is currently unimplemented.

This Node converts incoming `carma_v2x_msgs/BSM.msg` messages into
`carma_cooperative_perception_interfaces/DetectionList.msg` messages. Besides mapping `BSM.msg` message fields to
`DetectionList.msg` ones, this Node also projects the BSM's position data, represented in WGS-84 coordinates
(latitude, longitude, and elevation), to the local UTM zone’s coordinate frame.

> [!NOTE]\
> BSMs contain information about only one entity: the sending actor (vehicle or RSU), but this conversion Node outputs
> a detection list because that is the interface the multiple object fusion pipeline expects. The `DetectionList.msg`
> messages this conversion Node outputs will contain only one detection.

## Subscriptions

| Topic          | Message Type             | Description   |
| -------------- | ------------------------ | ------------- |
| `~/input/bsms` | `carma_v2x_msgs/BSM.msg` | Incoming BSMs |

## Publishers

| Topic                 | Message Type                                                | Frequency           | Description              |
| --------------------- | ----------------------------------------------------------- | ------------------- | ------------------------ |
| `~/output/detections` | `carma_cooperative_perception_interfaces/DetectionList.msg` | Subscription-driven | Outgoing detection lists |

## Parameters

| Topic                    | Data Type | Default Value | Required | Read Only | Description                                |
| ------------------------ | --------- | ------------- | -------- | --------- | ------------------------------------------ |
| `~/vehicle_motion_model` | `string`  | `''`          | Yes      | Yes       | The motion model assigned to incoming BSMs |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
