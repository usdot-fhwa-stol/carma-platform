# Trajectory executor Node

## Subscriptions

| Topic           | Message Type                               | Description |
| --------------- | ------------------------------------------ | ----------- |
| `~~/trajectory` | `carma_planning_msgs::msg::TrajectoryPlan` |             |
| `~/state`       | `carma_planning_msgs::msg::GuidanceState`  |             |

### Inherited from `carma_ros2_utils::CarmaLifecycleNode`

This Node does not inherit subscriptions from `carma_ros2_utils::CarmaLifecycleNode`.

## Publishers

| Topic                             | Message Type                               | Frequency    | Description |
| --------------------------------- | ------------------------------------------ | ------------ | ----------- |
| `~/<plugin_name>/plan_trajectory` | `carma_planning_msgs::msg::TrajectoryPlan` | Configurable |             |

### Inherited from

## Parameters

| Topic | Data Type | Default Value | Required | Read Only | Description |
| ----- | --------- | ------------- | -------- | --------- | ----------- |

### Inherited from

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
