# Motion computation Node

This Node is responsible for predicting environment objects' motion and sharing those predictions with the rest of
CARMA Platform. It serves three main functions:

- Convert incoming BSM, PSM, and mobility path data into `ExternalObject`s
- Predict incoming objects' motion using either the constant velocity (CV) or constant turn-rate and velocity (CTRV)
  model
- Queue and synchronize incoming object information so that it gets published all at once at a specific frequency

The following table lists the motion model used for each `ExternalObject` semantic category

| Object type     | Motion model |
| --------------- | ------------ |
| `UNKNOWN`       | CTRV         |
| `MOTORCYCLE`    | CTRV         |
| `SMALL_VEHICLE` | CTRV         |
| `LARGE_VEHICLE` | CTRV         |
| `PEDESTRIAN`    | CV           |
| default         | CV           |

> [!IMPORTANT]\
> This Node requires a valid georeference to do the motion predictions. Until it receives one on the `georeference`
> topic, it will not publish any.

> [!IMPORTANT]\
> This Node will only publish motion predictions when it receives external object information on the `external_objects`
> topic. Messages on the `incoming_*` topics will be queued until then.

## Subscriptions

| Topic                    | Message Type                                                                      | Description                                                                                                      |
| ------------------------ | --------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `external_objects`       | [`carma_perception_msgs::msg::ExternalObjectList`][external_object_list_msg_link] | Locally-detected objects                                                                                         |
| `incoming_mobility_path` | [`carma_v2x_msgs::msg::MobilityPath`][mobility_path_msg_link]                     | Self-reported planned motion paths from nearby vehicles                                                          |
| `incoming_bsm`           | [`carma_v2x_msgs::msg::BSM`][bsm_msg_link]                                        | Incoming basic safety messages                                                                                   |
| `incoming_psm`           | [`carma_v2x_msgs::msg::PSM`][psm_msg_link]                                        | Incoming personal safety messages                                                                                |
| `georeference`           | [`std_msgs::msg::String`][string_msg_link]                                        | Georeference point for projecting WGS-84 coordinates to a plane. Assumed and required to be a valid PROJ string. |

## Publishers

| Topic                         | Message Type                                                                      | Frequency           | Description                                                                                                                                                              |
| ----------------------------- | --------------------------------------------------------------------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `external_object_predictions` | [`carma_perception_msgs::msg::ExternalObjectList`][external_object_list_msg_link] | Subscription-driven | This Node will publish predictions only when it receives a message from the `external_objects` topic. Incoming mobility paths, BSMs, and PSMs will be queued until then. |

## Parameters

| Topic                             | Data Type | Default Value | Required | Read Only | Description                                                                                                                                                                                        |
| --------------------------------- | --------- | ------------- | -------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `prediction_time_step`            | `double`  | `0.1`         | Yes      | No        | Motion prediction time step (in seconds)                                                                                                                                                           |
| `mobility_path_time_step`         | `double`  | `0.1`         | Yes      | No        | Time between received mobility path predicted states (in seconds)                                                                                                                                  |
| `prediction_period`               | `double`  | `2.0`         | Yes      | No        | Prediction horizon (in seconds)                                                                                                                                                                    |
| `cv_x_accel_noise`                | `double`  | `9.0`         | Yes      | No        | CV Model X-Axis Acceleration Noise                                                                                                                                                                 |
| `cv_y_accel_noise`                | `double`  | `9.0`         | Yes      | No        | CV Model Y-Axis Acceleration Noise                                                                                                                                                                 |
| `prediction_process_noise_max`    | `double`  | `1000.0`      | Yes      | No        | Maximum expected process noise; used for mapping noise to confidence in _[0,1]_ range                                                                                                              |
| `prediction_confidence_drop_rate` | `double`  | `0.95`        | Yes      | No        | Percentage of initial confidence to propagate to next time step                                                                                                                                    |
| `enable_bsm_processing`           | `bool`    | `false`       | Yes      | No        | If `true` then BSM messages will be converted to `ExternalObjects`. If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)            |
| `enable_psm_processing`           | `bool`    | `false`       | Yes      | No        | If `true` then PSM messages will be converted to `ExternalObjects`. If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)            |
| `enable_mobility_path_processing` | `bool`    | `true`        | Yes      | No        | If `true` then MobilityPath messages will be converted to `ExternalObjects`. If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)   |
| `enable_sensor_processing`        | `bool`    | `false`       | Yes      | No        | If `true` then `ExternalObjects` generated from sensor data will be processed. If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated) |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.

[external_object_list_msg_link]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_perception_msgs/msg/ExternalObjectList.msg
[mobility_path_msg_link]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_v2x_msgs/msg/MobilityPath.msg
[bsm_msg_link]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_v2x_msgs/msg/BSM.msg
[psm_msg_link]: https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/carma_v2x_msgs/msg/PSM.msg
[string_msg_link]: http://docs.ros.org/en/api/std_msgs/html/msg/String.html
