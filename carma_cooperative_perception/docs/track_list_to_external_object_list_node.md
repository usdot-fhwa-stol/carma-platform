# Track to external object list

This Node generates `carma_perception_msgs/ExternalObjectList.msg` messages
from the tracking pipeline's outputted
`carma_cooperative_perception_interfaces/TrackList.msg` messages. We designed
`carma_cooperative_perception` package to be transparent to CARMA Platform, so
we need this conversion Node to inject the fused obstacle data back into CARMA
Platform's perception pipeline.

> [!IMPORTANT]\
> The `carma_cooperative_perception_interfaces/msg/Track.msg` messages store
> tracks' identifiers (IDs) as strings, but the
> `carma_perception_msgs/msg/ExternalObject.msg` messages use unsigned
> integers for the `id` field (which is different than the `bsm_id` field). If
> the string-to-integer conversion fails during message conversion, the
> resulting `ExternalObject`'s `id` field will be unpopulated.

## Subscriptions

| Topic                | Message Type                                                              | Description     |
| -------------------- | ------------------------------------------------------------------------- | --------------- |
| `~/input/track_list` | [`carma_cooperative_perception_interfaces/TrackList.msg`][track_list_msg] | Incoming tracks |

## Publishers

| Topic                           | Message Type                                                               | Frequency           | Description                            |
| ------------------------------- | -------------------------------------------------------------------------- | ------------------- | -------------------------------------- |
| `~/output/external_object_list` | [`carma_perception_msgs/ExternalObjectList.msg`][external_object_list_msg] | Subscription-driven | External objects generated from tracks |

## Parameters

This Node does not provide parameters.

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.

[track_list_msg]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_cooperative_perception_interfaces/msg/TrackList.msg
[external_object_list_msg]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_perception_msgs/msg/ExternalObjectList.msg
