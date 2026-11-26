# Pure pursuit wrapper Node

This Node wraps the Autoware.Auto `pure_pursuit` Node.

## Subscriptions

This Node does not provide additional subscriptions.

### Inherited from `carma_guidance_plugins::ControlPlugin`

| Topic                           | Message Type                               | Description |
| ------------------------------- | ------------------------------------------ | ----------- |
| `~/current_pose`                | `geometry_msgs::msg::PoseStamped`          |             |
| `~/vehicle/twist`               | `geometry_msgs::msg::TwistStamped`         |             |
| `~/<node_name>/plan_trajectory` | `carma_planning_msgs::msg::TrajectoryPlan` |             |

### Inherited from `carma_guidance_plugins::PluginBaseNode`

This Node does not inherit subscriptions from `carma_guidance_plugins::PluginBaseNode`.

### Inherited from `carma_ros2_utils::CarmaLifecycleNode`

This Node does not inherit subscriptions from `carma_ros2_utils::CarmaLifecycleNode`.

## Publishers

This Node does not provide additional publishers.

### Inherited from `carma_guidance_plugins::ControlPlugin`

| Topic        | Message Type                                | Frequency | Description |
| ------------ | ------------------------------------------- | --------- | ----------- |
| `~/ctrl_raw` | `autoware_msgs::msg::ControlCommandStamped` | 30 Hz     |             |

### Inherited from `carma_guidance_plugins::PluginBaseNode`

| Topic                | Message Type                       | Frequency | Description |
| -------------------- | ---------------------------------- | --------- | ----------- |
| `~/plugin_discovery` | `carma_planning_msgs::msg::Plugin` | 2 Hz      |             |

### Inherited from `carma_ros2_utils::CarmaLifecycleNode`

| Topic           | Message Type                   | Frequency    | Description |
| --------------- | ------------------------------ | ------------ | ----------- |
| `/system_alert` | `carma_msgs::msg::SystemAlert` | Event-driven |             |
