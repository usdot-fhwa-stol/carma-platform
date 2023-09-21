# Roadway objects Node

This Node filters incoming external objects to remove any detected objects that are currently off the roadway.

## Subscriptions

| Topic                | Message Type                                                                 | Description               |
| -------------------- | ---------------------------------------------------------------------------- | ------------------------- |
| `~/external_objects` | [`carma_perception_msgs::msg::ExternalObjectList`][external_object_list_msg] | Incoming external objects |

[external_object_list_msg]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_perception_msgs/msg/ExternalObjectList.msg

## Publishers

| Topic               | Message Type                                                               | Frequency           | Description                                     |
| ------------------- | -------------------------------------------------------------------------- | ------------------- | ----------------------------------------------- |
| `~/roadway_objects` | [`carma_perception_msgs::msg::RoadwayObstacleList`][roadway_obstacle_list] | Subscription-driven | External objects that are currently on the road |

[roadway_obstacle_list]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_perception_msgs/msg/RoadwayObstacleList.msg

## Parameters

This Node does not provide parameters.

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
