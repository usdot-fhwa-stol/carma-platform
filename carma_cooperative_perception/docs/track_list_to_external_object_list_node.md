# Track to external object list

This Node generates `carma_perception_msgs/ExternalObjectList.msg` messages from the tracking pipeline's outputted `carma_cooperative_perception_interfaces/TrackList.msg` messages. We designed `carma_cooperative_perception` package to be transparent to CARMA Platform, so we need this conversion Node to inject the fused obstacle data back into CARMA Platform's perception pipeline. Similar to the other conversion nodes, this Node performs coordinate changed but in reverse. It uses the host vehicle's current pose to convert the object position data into the host vehicle's coordinate frame.

## Subscriptions

| Topic            | Message Type                                            | Description     |
| ---------------- | ------------------------------------------------------- | --------------- |
| `~/input/tracks` | `carma_cooperative_perception_interfaces/TrackList.msg` | Incoming tracks |

## Publishers

| Topic                       | Message Type                                            | Frequency           | Description                            |
| --------------------------- | ------------------------------------------------------- | ------------------- | -------------------------------------- |
| `~/output/external_objects` | `carma_cooperative_perception_interfaces/TrackList.msg` | Subscription-driven | External objects generated from tracks |

## Parameters

| Topic                      | Data Type | Default Value | Required | Read Only | Description                                |
| -------------------------- | --------- | ------------- | -------- | --------- | ------------------------------------------ |
| `~/execution_frequency_hz` | `double`  | `10.0`        | Yes      | Yes       | Pipeline execution frequency in Hertz (Hz) |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
