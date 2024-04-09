# External object list to SDSM Node

This Node generates `carma_v2x_msgs/SensorDataSharingMessage.msg` messages from the
`carma_perception_msgs/ExternalObjectList.msg` messages that the host’s local perception stack outputs. It uses the
host vehicle's current position to convert the objects' positions into the appropriate coordinate frames. The host
vehicle's position becomes the reference point, and the objects' positions become offsets. This Node allows us to share
the tracking pipeline's outputs with neighboring vehicles and infrastructure, enabling and facilitating cooperative
perception.

We don’t want to include data fused from incoming BSMs or SDSMs in our own SDSM because it would violate the implicit
assumption that measurements in SDSMs are independent among broadcasting actors. Quoting again a relevant portion from
SAE J3224 Section 5.2.1,

> In the object detection state an HV or HRSU identifies objects in its field of view using its sensors and determines
> the static and dynamic characteristics of those detected objects.

We interpret this to mean SDSMs should contain detection objects perceived only with an actor’s local capabilities. The
actor can perform object-level fusion before generating the SDSM, but the fusion process must involve only objects
detected with the host’s local perception capabilities.

## Subscriptions

| Topic                      | Message Type                                   | Description                                 |
| -------------------------- | ---------------------------------------------- | ------------------------------------------- |
| `~/input/external_objects` | `carma_perception_msgs/ExternalObjectList.msg` | Incoming external objects perceived locally |

## Publishers

| Topic            | Message Type                                  | Frequency           | Description                                   |
| ---------------- | --------------------------------------------- | ------------------- | --------------------------------------------- |
| `~/output/sdsms` | `carma_v2x_msgs/SensorDataSharingMessage.msg` | Subscription-driven | SDSMs generated from the external object list |

## Parameters

This node does provide parameters.

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
