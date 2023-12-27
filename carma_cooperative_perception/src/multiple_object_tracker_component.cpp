// Copyright 2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carma_cooperative_perception/multiple_object_tracker_component.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <units.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <multiple_object_tracking/clustering.hpp>
#include <multiple_object_tracking/ctra_model.hpp>
#include <multiple_object_tracking/ctrv_model.hpp>
#include <multiple_object_tracking/fusing.hpp>
#include <multiple_object_tracking/gating.hpp>
#include <multiple_object_tracking/scoring.hpp>
#include <multiple_object_tracking/temporal_alignment.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace multiple_object_tracking
{
auto two_dimensional_distance(const CtrvState & lhs, const CtrvState & rhs) -> float
{
  const auto x_diff_sq{std::pow(remove_units(lhs.position_x) - remove_units(rhs.position_x), 2)};
  const auto y_diff_sq{std::pow(remove_units(lhs.position_y) - remove_units(rhs.position_y), 2)};

  return std::sqrt(x_diff_sq + y_diff_sq);
}

auto two_dimensional_distance(const CtraState & lhs, const CtraState & rhs) -> float
{
  const auto x_diff_sq{std::pow(remove_units(lhs.position_x) - remove_units(rhs.position_x), 2)};
  const auto y_diff_sq{std::pow(remove_units(lhs.position_y) - remove_units(rhs.position_y), 2)};

  return std::sqrt(x_diff_sq + y_diff_sq);
}

auto weighted_distance(const CtrvState & lhs, const CtrvState & rhs) -> float
{
  const auto x_diff_sq{std::pow(remove_units(lhs.position_x) - remove_units(rhs.position_x), 2)};
  const auto y_diff_sq{std::pow(remove_units(lhs.position_y) - remove_units(rhs.position_y), 2)};
  const auto v_diff_sq{100 * std::pow(remove_units(lhs.velocity) - remove_units(rhs.velocity), 2)};

  return std::sqrt(x_diff_sq + y_diff_sq + v_diff_sq);
}

auto weighted_distance(const CtraState & lhs, const CtraState & rhs) -> float
{
  const auto x_diff_sq{std::pow(remove_units(lhs.position_x) - remove_units(rhs.position_x), 2)};
  const auto y_diff_sq{std::pow(remove_units(lhs.position_y) - remove_units(rhs.position_y), 2)};
  const auto v_diff_sq{100 * std::pow(remove_units(lhs.velocity) - remove_units(rhs.velocity), 2)};

  return std::sqrt(x_diff_sq + y_diff_sq + v_diff_sq);
}
}  // namespace multiple_object_tracking

namespace carma_cooperative_perception
{

namespace mot = multiple_object_tracking;

auto make_semantic_class(std::size_t numeric_value)
{
  switch (numeric_value) {
    case 0:
      return mot::SemanticClass::kUnknown;
    case 1:
      return mot::SemanticClass::kSmallVehicle;
    case 2:
      return mot::SemanticClass::kLargeVehicle;
    case 3:
      return mot::SemanticClass::kPedestrian;
    case 4:
      return mot::SemanticClass::kMotorcycle;
  }

  return mot::SemanticClass::kUnknown;
}

auto semantic_class_to_numeric_value(mot::SemanticClass semantic_class)
{
  switch (semantic_class) {
    case mot::SemanticClass::kUnknown:
      return 0;
    case mot::SemanticClass::kSmallVehicle:
      return 1;
    case mot::SemanticClass::kLargeVehicle:
      return 2;
    case mot::SemanticClass::kPedestrian:
      return 3;
    case mot::SemanticClass::kMotorcycle:
      return 4;
  }

  return 0;
}

auto make_ctrv_detection(const carma_cooperative_perception_interfaces::msg::Detection & msg)
  -> Detection
{
  const auto timestamp{
    units::time::second_t{static_cast<double>(msg.header.stamp.sec)} +
    units::time::nanosecond_t{static_cast<double>(msg.header.stamp.nanosec)}};

  tf2::Quaternion orientation;
  orientation.setX(msg.pose.pose.orientation.x);
  orientation.setY(msg.pose.pose.orientation.y);
  orientation.setZ(msg.pose.pose.orientation.z);
  orientation.setW(msg.pose.pose.orientation.w);

  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};

  tf2::Matrix3x3 matrix{orientation};
  matrix.getRPY(roll, pitch, yaw);

  const mot::CtrvState state{
    units::length::meter_t{msg.pose.pose.position.x},
    units::length::meter_t{msg.pose.pose.position.y},
    units::velocity::meters_per_second_t{msg.twist.twist.linear.x},
    mot::Angle{units::angle::radian_t{yaw}},
    units::angular_velocity::radians_per_second_t{msg.twist.twist.angular.z}};

  mot::CtrvStateCovariance covariance = mot::CtrvStateCovariance::Zero();
  covariance(0, 0) = msg.pose.covariance.at(0);
  covariance(1, 1) = msg.pose.covariance.at(7);
  covariance(2, 2) = msg.twist.covariance.at(0);
  covariance(3, 3) = msg.pose.covariance.at(35);
  covariance(4, 4) = msg.twist.covariance.at(35);

  return mot::CtrvDetection{
    timestamp, state, covariance, mot::Uuid{msg.id}, make_semantic_class(msg.semantic_class)};
}

auto make_ctra_detection(const carma_cooperative_perception_interfaces::msg::Detection & msg)
  -> Detection
{
  const auto timestamp{
    units::time::second_t{static_cast<double>(msg.header.stamp.sec)} +
    units::time::nanosecond_t{static_cast<double>(msg.header.stamp.nanosec)}};

  tf2::Quaternion orientation;
  orientation.setX(msg.pose.pose.orientation.x);
  orientation.setY(msg.pose.pose.orientation.y);
  orientation.setZ(msg.pose.pose.orientation.z);
  orientation.setW(msg.pose.pose.orientation.w);

  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};

  tf2::Matrix3x3 matrix{orientation};
  matrix.getRPY(roll, pitch, yaw);

  const mot::CtraState state{
    units::length::meter_t{msg.pose.pose.position.x},
    units::length::meter_t{msg.pose.pose.position.y},
    units::velocity::meters_per_second_t{msg.twist.twist.linear.x},
    mot::Angle{units::angle::radian_t{yaw}},
    units::angular_velocity::radians_per_second_t{msg.twist.twist.angular.z},
    units::acceleration::meters_per_second_squared_t{msg.accel.accel.linear.x}};

  mot::CtraStateCovariance covariance = mot::CtraStateCovariance::Zero();
  covariance(0, 0) = msg.pose.covariance.at(0);
  covariance(1, 1) = msg.pose.covariance.at(7);
  covariance(2, 2) = msg.twist.covariance.at(0);
  covariance(3, 3) = msg.pose.covariance.at(35);
  covariance(4, 4) = msg.twist.covariance.at(35);
  covariance(5, 5) = msg.accel.covariance.at(0);

  return mot::CtraDetection{
    timestamp, state, covariance, mot::Uuid{msg.id}, make_semantic_class(msg.semantic_class)};
}

auto make_detection(const carma_cooperative_perception_interfaces::msg::Detection & msg)
  -> Detection
{
  switch (msg.motion_model) {
    case msg.MOTION_MODEL_CTRV:
      return make_ctrv_detection(msg);

    case msg.MOTION_MODEL_CTRA:
      return make_ctra_detection(msg);

    case msg.MOTION_MODEL_CV:
      throw std::runtime_error("unsupported motion model type '3: constant velocity (CV)'");
  }

  throw std::runtime_error("unkown motion model type '" + std::to_string(msg.motion_model) + "'");
}

static auto to_ros_msg(const mot::CtraTrack & track)
{
  carma_cooperative_perception_interfaces::msg::Track msg;

  msg.header.stamp.sec = mot::remove_units(units::math::floor(track.timestamp));
  msg.header.stamp.nanosec = mot::remove_units(
    units::time::nanosecond_t{units::math::fmod(track.timestamp, units::time::second_t{1.0})});

  msg.header.frame_id = "map";

  msg.id = track.uuid.value();
  msg.motion_model = msg.MOTION_MODEL_CTRA;
  msg.pose.pose.position.x = mot::remove_units(track.state.position_x);
  msg.pose.pose.position.y = mot::remove_units(track.state.position_y);

  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, mot::remove_units(track.state.yaw.get_angle()));
  msg.pose.pose.orientation.x = orientation.getX();
  msg.pose.pose.orientation.y = orientation.getY();
  msg.pose.pose.orientation.z = orientation.getZ();
  msg.pose.pose.orientation.w = orientation.getW();

  msg.twist.twist.linear.x = mot::remove_units(track.state.velocity);
  msg.twist.twist.angular.z = mot::remove_units(track.state.yaw_rate);

  msg.accel.accel.linear.x = mot::remove_units(track.state.acceleration);

  msg.semantic_class = semantic_class_to_numeric_value(mot::get_semantic_class(track));

  return msg;
}

static auto to_ros_msg(const mot::CtrvTrack & track)
{
  carma_cooperative_perception_interfaces::msg::Track msg;

  msg.header.stamp.sec = mot::remove_units(units::math::floor(track.timestamp));
  msg.header.stamp.nanosec = mot::remove_units(
    units::time::nanosecond_t{units::math::fmod(track.timestamp, units::time::second_t{1.0})});
  msg.header.frame_id = "map";

  msg.id = track.uuid.value();
  msg.motion_model = msg.MOTION_MODEL_CTRV;
  msg.pose.pose.position.x = mot::remove_units(track.state.position_x);
  msg.pose.pose.position.y = mot::remove_units(track.state.position_y);

  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, mot::remove_units(track.state.yaw.get_angle()));
  msg.pose.pose.orientation.x = orientation.getX();
  msg.pose.pose.orientation.y = orientation.getY();
  msg.pose.pose.orientation.z = orientation.getZ();
  msg.pose.pose.orientation.w = orientation.getW();

  msg.twist.twist.linear.x = mot::remove_units(track.state.velocity);
  msg.twist.twist.angular.z = mot::remove_units(track.state.yaw_rate);

  msg.semantic_class = semantic_class_to_numeric_value(mot::get_semantic_class(track));

  return msg;
}

static auto to_ros_msg(const Track & track)
{
  static constexpr mot::Visitor visitor{
    [](const mot::CtrvTrack & t) { return to_ros_msg(t); },
    [](const mot::CtraTrack & t) { return to_ros_msg(t); },
    [](const auto &) { throw std::runtime_error{"cannot make ROS 2 message from track type"}; }};

  return std::visit(visitor, track);
}

MultipleObjectTrackerNode::MultipleObjectTrackerNode(const rclcpp::NodeOptions & options)
: CarmaLifecycleNode{options}
{
}

auto MultipleObjectTrackerNode::handle_on_configure(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: configuring");

  track_list_pub_ = create_publisher<carma_cooperative_perception_interfaces::msg::TrackList>(
    "output/track_list", 1);

  detection_list_sub_ = create_subscription<
    carma_cooperative_perception_interfaces::msg::DetectionList>(
    "input/detection_list", 1,
    [this](const carma_cooperative_perception_interfaces::msg::DetectionList::SharedPtr msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        this->store_new_detections(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->detection_list_sub_->get_topic_name(), current_state.c_str());
      }
    });

  declare_parameter(
    "execution_frequency_hz", mot::remove_units(units::frequency::hertz_t{1 / execution_period_}));

  declare_parameter(
    "track_promotion_threshold", static_cast<int>(track_manager_.get_promotion_threshold().value));

  declare_parameter(
    "track_removal_threshold", static_cast<int>(track_manager_.get_promotion_threshold().value));

  on_set_parameters_callback_ =
    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";

      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "execution_frequency_hz") {
          if (this->get_current_state().label() == "active") {
            result.successful = false;
            result.reason = "parameter is read-only while node is in 'Active' state";

            RCLCPP_ERROR(
              get_logger(), "Cannot change parameter 'execution_frequency_hz': " + result.reason);

            break;
          } else {
            this->execution_period_ = 1 / units::frequency::hertz_t{parameter.as_double()};
          }
        } else if (parameter.get_name() == "track_promotion_threshold") {
          if (this->get_current_state().label() == "active") {
            result.successful = false;
            result.reason = "parameter is read-only while node is in 'Active' state";

            RCLCPP_ERROR(
              get_logger(), "Cannot change parameter 'execution_frequency_hz': " + result.reason);

            break;
          } else {
            if (const auto value{parameter.as_int()}; value < 0) {
              result.successful = false;
              result.reason = "parameter must be nonnegative";
            } else {
              this->track_manager_.set_promotion_threshold_and_update(
                mot::PromotionThreshold{static_cast<std::size_t>(value)});
            }
          }
        } else if (parameter.get_name() == "track_removal_threshold") {
          if (this->get_current_state().label() == "active") {
            result.successful = false;
            result.reason = "parameter is read-only while node is in 'Active' state";

            RCLCPP_ERROR(
              get_logger(), "Cannot change parameter 'execution_frequency_hz': " + result.reason);

            break;
          } else {
            if (const auto value{parameter.as_int()}; value < 0) {
              result.successful = false;
              result.reason = "parameter must be nonnegative";
            } else {
              this->track_manager_.set_removal_threshold_and_update(
                mot::RemovalThreshold{static_cast<std::size_t>(value)});
            }
          }
        } else {
          result.successful = false;
          result.reason = "Unexpected parameter name '" + parameter.get_name() + '\'';
        }
      }

      return result;
    });

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully configured");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_activate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: activating");

  if (pipeline_execution_timer_ != nullptr) {
    // The user might have changed the timer period since last time the Node was active
    pipeline_execution_timer_->reset();
  }

  const std::chrono::duration<double, std::nano> period_ns{mot::remove_units(execution_period_)};
  // pipeline_execution_timer_ = create_wall_timer(period_ns, [this] { execute_pipeline(); });
  pipeline_execution_timer_ =
    rclcpp::create_timer(this, this->get_clock(), period_ns, [this] { execute_pipeline(); });

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully activated");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: deactivating");

  // There is currently no way to change a timer's period in ROS 2, so we will
  // have to create a new one in case a user changes the period.
  pipeline_execution_timer_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully deactivated");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: cleaning up");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();

  undeclare_parameter("execution_frequency_hz");
  remove_on_set_parameters_callback(on_set_parameters_callback_.get());

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully cleaned up");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: shutting down");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully shut down");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::store_new_detections(
  const carma_cooperative_perception_interfaces::msg::DetectionList & msg) -> void
{
  if (std::size(msg.detections) == 0) {
    RCLCPP_WARN(this->get_logger(), "Not storing detections: incoming detection list is empty");
    return;
  }

  for (const auto & detection_msg : msg.detections) {
    try {
      const auto detection{make_detection(detection_msg)};
      const auto uuid{mot::get_uuid(detection)};

      if (uuid_index_map_.count(uuid) == 0) {
        detections_.push_back(std::move(detection));
        uuid_index_map_[uuid] = std::size(detections_) - 1;
        RCLCPP_WARN_STREAM(this->get_logger(), "Storing detection with ID '" << uuid << "'.");
      } else {
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Detection with ID '" << uuid << "' already exists. Overwriting its data");
        detections_.at(uuid_index_map_[uuid]) = detection;
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR(
        this->get_logger(), "Ignoring detection with ID '%s': %s", detection_msg.id.c_str(),
        error.what());
    }
  }
}

static auto temporally_align_detections(
  std::vector<Detection> & detections, units::time::second_t end_time) -> void
{
  for (auto & detection : detections) {
    // mot::propagate_to_time(detection, end_time, mot::default_unscented_transform);
    mot::propagate_to_time(detection, end_time, mot::UnscentedTransform{1.0, 2.0, 0.0});
  }
}

static auto predict_track_states(std::vector<Track> tracks, units::time::second_t end_time)
{
  for (auto & track : tracks) {
    // mot::propagate_to_time(track, end_time, mot::default_unscented_transform);
    mot::propagate_to_time(track, end_time, mot::UnscentedTransform{1.0, 2.0, 0.0});
  }

  return tracks;
}

struct WeightedDistanceScore
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      return weighted_distance(track.state, detection.state);
    } else {
      return std::nullopt;
    }
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & t, const auto & d) { return (*this)(t, d); }, track, detection);
  }
};

struct WeightedMahalanobisDistanceScore
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      const auto velocity_distance{
        std::abs(mot::remove_units(track.state.velocity - detection.state.velocity))};
      return mot::mahalanobis_distance(track.state, track.covariance, detection.state) +
             10 * velocity_distance;
    } else {
      return std::nullopt;
    }
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & t, const auto & d) { return (*this)(t, d); }, track, detection);
  }
};

struct SemanticMahalanobisDistanceScore
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      const auto dist{mot::mahalanobis_distance(track.state, track.covariance, detection.state)};
      if (
        track.semantic_class == mot::SemanticClass::kUnknown ||
        detection.semantic_class == mot::SemanticClass::kUnknown) {
        return dist;
      }

      if (track.semantic_class != detection.semantic_class) {
        return std::nullopt;
      }

      return dist;
    } else {
      return std::nullopt;
    }
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & t, const auto & d) { return (*this)(t, d); }, track, detection);
  }
};

struct SemanticTwoDimensionalDistanceScore
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      const auto dist{mot::two_dimensional_distance(track.state, detection.state)};
      if (
        track.semantic_class == mot::SemanticClass::kUnknown ||
        detection.semantic_class == mot::SemanticClass::kUnknown) {
        return dist;
      }

      if (track.semantic_class != detection.semantic_class) {
        return std::nullopt;
      }

      return dist;
    } else {
      return std::nullopt;
    }
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & t, const auto & d) { return (*this)(t, d); }, track, detection);
  }
};

// struct SemanticTwoDimensionalMahalanobisDistanceScore
// {
//   template <typename Track, typename Detection>
//   auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
//   {
//     if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
//       const auto dist{mot::two_dimensional_mahalanobis_distance(track.state, detection.state)};
//       if (
//         track.semantic_class == mot::SemanticClass::kUnknown ||
//         detection.semantic_class == mot::SemanticClass::kUnknown) {
//         return dist;
//       }

//       if (track.semantic_class != detection.semantic_class) {
//         return std::nullopt;
//       }

//       return dist;
//     } else {
//       return std::nullopt;
//     }
//   }

//   template <typename... TrackAlternatives, typename... DetectionAlternatives>
//   auto operator()(
//     const std::variant<TrackAlternatives...> & track,
//     const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
//   {
//     return std::visit(
//       [this](const auto & t, const auto & d) { return (*this)(t, d); }, track, detection);
//   }
// };

struct CustomMetric
{
  template <typename Detection>
  auto operator()(const mot::Point & point, const Detection & detection) const -> double
  {
    double sum{0};

    sum += std::pow(mot::remove_units(point.position_x - detection.state.position_x), 2);
    sum += std::pow(mot::remove_units(point.position_y - detection.state.position_y), 2);

    const auto p_yaw_rad{mot::remove_units(point.yaw.get_angle())};
    const auto d_yaw_rad{mot::remove_units(detection.state.yaw.get_angle())};
    const auto abs_diff{std::abs(p_yaw_rad - d_yaw_rad)};
    sum += std::min(abs_diff, 2 * 3.14159265359 - abs_diff);

    return sum;
  }

  template <typename... Alternatives>
  auto operator()(mot::Point point, const std::variant<Alternatives...> & detection) const -> double
  {
    const mot::Visitor visitor{
      [this](const mot::Point & p, const mot::CtrvDetection & d) { return this->operator()(p, d); },
      [this](const mot::Point & p, const mot::CtraDetection & d) { return this->operator()(p, d); },
      [](const mot::Point &, const auto &) { return std::numeric_limits<double>::max(); }};

    return std::visit(visitor, std::variant<mot::Point>{point}, detection);
  }
};

template <typename TrackList>
auto prune_scores_if(mot::ScoreMap & scores, const TrackList & tracks)
{
  std::map<mot::Uuid, mot::SemanticClass> semantic_class_map;
  for (const auto track : tracks) {
    semantic_class_map[mot::get_uuid(track)] = mot::get_semantic_class(track);
  }

  std::vector<std::pair<mot::Uuid, mot::Uuid>> keys_to_prune;

  for (const auto & [key_pair, score] : scores) {
    // Assuming both keys have same semantic class
    const auto semantic_class{semantic_class_map.at(key_pair.first)};

    if (semantic_class == mot::SemanticClass::kMotorcycle) {
      if (score > 7.5) {
        keys_to_prune.emplace_back(key_pair);
      }
    } else {
      if (score > 5) {
        keys_to_prune.emplace_back(key_pair);
      }
    }
  }

  for (const auto & key : keys_to_prune) {
    scores.erase(key);
  }
}

class CustomHasAssociation
{
public:
  explicit CustomHasAssociation(const mot::AssociationMap & associations)
  {
    for (const auto & [track_uuid, detection_uuids] : associations) {
      associated_track_uuids_.insert(track_uuid);
      associated_detection_uuids_.insert(std::cbegin(detection_uuids), std::cend(detection_uuids));
    }
  }

  template <typename Track>
  auto has_association_track(const Track & track) const -> bool
  {
    return associated_track_uuids_.count(mot::get_uuid(track)) != 0;
  }

  template <typename Detection>
  auto has_association_detection(const Detection & detection) const -> bool
  {
    return associated_detection_uuids_.count(mot::get_uuid(detection)) != 0;
  }

private:
  std::unordered_set<mot::Uuid> associated_track_uuids_;
  std::unordered_set<mot::Uuid> associated_detection_uuids_;
};

auto MultipleObjectTrackerNode::execute_pipeline() -> void
{
  try {
    // Logging only
    RCLCPP_ERROR(get_logger(), "EXECUTING PIPELINE");

    static constexpr mot::Visitor make_track_visitor{
      [](const mot::CtrvDetection & d) { return Track{mot::make_track<mot::CtrvTrack>(d)}; },
      [](const mot::CtraDetection & d) { return Track{mot::make_track<mot::CtraTrack>(d)}; },
      [](const auto &) { throw std::runtime_error("cannot make track from given detection"); },
    };

    if (track_manager_.get_all_tracks().empty()) {
      RCLCPP_DEBUG(
        get_logger(), "List of tracks is empty. Converting detections to tentative tracks");

      const auto clusters{mot::cluster_detections(detections_, 0.75)};
      for (const auto & cluster : clusters) {
        const auto detection{std::cbegin(cluster.get_detections())->second};
        track_manager_.add_tentative_track(std::visit(make_track_visitor, detection));
      }

      track_list_pub_->publish(carma_cooperative_perception_interfaces::msg::TrackList{});

      detections_.clear();
      uuid_index_map_.clear();
      return;
    }

    const units::time::second_t current_time{this->now().seconds()};

    // Logging only
    static constexpr mot::Visitor state_to_string{
      [](mot::CtrvDetection const & detection) {
        std::stringstream ss;
        ss << "CtrvState: \n";
        ss << "uuid: " << mot::get_uuid(detection) << '\n';
        ss << "time: " << detection.timestamp << '\n';
        ss << "x: " << detection.state.position_x << '\n';
        ss << "y: " << detection.state.position_y << '\n';
        ss << "velocity: " << detection.state.velocity << '\n';
        ss << "yaw: " << detection.state.yaw.get_angle() << '\n';
        ss << "yaw_rate: " << detection.state.yaw_rate << '\n';
        ss << "covariance: " << detection.covariance << '\n';
        ss << "semantic class: " << semantic_class_to_numeric_value(detection.semantic_class)
           << '\n';

        return ss.str();
      },
      [](mot::CtraDetection const & detection) {
        std::stringstream ss;
        ss << "CtraState: \n";
        ss << "uuid: " << mot::get_uuid(detection) << '\n';
        ss << "time: " << detection.timestamp << '\n';
        ss << "x: " << detection.state.position_x << '\n';
        ss << "y: " << detection.state.position_y << '\n';
        ss << "velocity: " << detection.state.velocity << '\n';
        ss << "yaw: " << detection.state.yaw.get_angle() << '\n';
        ss << "yaw_rate: " << detection.state.yaw_rate << '\n';
        ss << "acceleration: " << detection.state.acceleration << '\n';
        ss << "covariance: " << detection.covariance << '\n';
        ss << "semantic class: " << semantic_class_to_numeric_value(detection.semantic_class)
           << '\n';

        return ss.str();
      }};

    // Logging only
    static constexpr mot::Visitor track_state_to_string{
      [](mot::CtrvTrack const & detection) {
        std::stringstream ss;
        ss << "CtrvState: \n";
        ss << "uuid: " << mot::get_uuid(detection) << '\n';
        ss << "time: " << detection.timestamp << '\n';
        ss << "x: " << detection.state.position_x << '\n';
        ss << "y: " << detection.state.position_y << '\n';
        ss << "velocity: " << detection.state.velocity << '\n';
        ss << "yaw: " << detection.state.yaw.get_angle() << '\n';
        ss << "yaw_rate: " << detection.state.yaw_rate << '\n';
        ss << "covariance: " << detection.covariance << '\n';
        ss << "semantic class: " << semantic_class_to_numeric_value(detection.semantic_class)
           << '\n';

        return ss.str();
      },
      [](mot::CtraTrack const & detection) {
        std::stringstream ss;
        ss << "CtraState: \n";
        ss << "uuid: " << mot::get_uuid(detection) << '\n';
        ss << "time: " << detection.timestamp << '\n';
        ss << "x: " << detection.state.position_x << '\n';
        ss << "y: " << detection.state.position_y << '\n';
        ss << "velocity: " << detection.state.velocity << '\n';
        ss << "yaw: " << detection.state.yaw.get_angle() << '\n';
        ss << "yaw_rate: " << detection.state.yaw_rate << '\n';
        ss << "acceleration: " << detection.state.acceleration << '\n';
        ss << "covariance: " << detection.covariance << '\n';
        ss << "semantic class: " << semantic_class_to_numeric_value(detection.semantic_class)
           << '\n';

        return ss.str();
      }};

    // Logging only
    for (auto const & detection : detections_) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Detection (pre): " << std::visit(state_to_string, detection));
    }

    temporally_align_detections(detections_, current_time);

    // Logging only
    for (auto const & detection : detections_) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Detection (post): " << std::visit(state_to_string, detection));
    }

    // Logging only
    for (auto const & track : track_manager_.get_all_tracks()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Track: " << std::visit(track_state_to_string, track));
    }

    const auto predicted_tracks{
      predict_track_states(track_manager_.get_all_tracks(), current_time)};
    // auto scores{
    //   mot::score_tracks_and_detections(predicted_tracks, detections_, mot::euclidean_distance_score)};
    // const auto scores{
    //   mot::score_tracks_and_detections(predicted_tracks, detections_, WeightedDistanceScore{})};
    // const auto scores{mot::score_tracks_and_detections(
    //   predicted_tracks, detections_, mot::mahalanobis_distance_score)};
    // const auto scores{mot::score_tracks_and_detections(
    //   predicted_tracks, detections_, WeightedMahalanobisDistanceScore{})};
    // const auto scores{mot::score_tracks_and_detections(
    //   predicted_tracks, detections_, SemanticMahalanobisDistanceScore{})};
    auto scores{mot::score_tracks_and_detections(
      predicted_tracks, detections_, SemanticTwoDimensionalDistanceScore{})};
    // auto scores{mot::score_tracks_and_detections(
    //   predicted_tracks, detections_, SemanticTwoDimensionalMahalanobisDistanceScore{})};

    const auto weighted_scores{
      mot::score_tracks_and_detections(predicted_tracks, detections_, WeightedDistanceScore{})};

    const auto mahalanobis_scores{mot::score_tracks_and_detections(
      predicted_tracks, detections_, mot::mahalanobis_distance_score)};

    {
      std::stringstream ss;
      ss << "Scores (pre filter): \n";
      for (const auto & [uuid_pair, score] : scores) {
        ss << uuid_pair.first << ", " << uuid_pair.second << ": " << score << '\n';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    // mot::prune_track_and_detection_scores_if(
    //   scores, [](const auto & score) { return score > 5.0; });

    prune_scores_if(scores, track_manager_.get_all_tracks());

    {
      std::stringstream ss;
      ss << "Scores (post filter): \n";
      for (const auto & [uuid_pair, score] : scores) {
        ss << uuid_pair.first << ", " << uuid_pair.second << ": " << score << '\n';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    const auto associations{
      mot::associate_detections_to_tracks(scores, mot::gnn_association_visitor)};

    // Logging only
    {
      std::stringstream ss;
      ss << "Current detections: ";
      for (const auto & detection : detections_) {
        ss << mot::get_uuid(detection) << ' ';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Current tracks: ";
      for (const auto & track : track_manager_.get_all_tracks()) {
        ss << mot::get_uuid(track) << ' ';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    CustomHasAssociation custom_has_ass{associations};

    {
      std::stringstream ss;
      ss << "Associated detections (custom): ";
      for (const auto & detection : detections_) {
        if (custom_has_ass.has_association_detection(detection)) {
          ss << mot::get_uuid(detection) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Associated tracks (custom): ";
      for (const auto & track : track_manager_.get_all_tracks()) {
        if (custom_has_ass.has_association_track(track)) {
          ss << mot::get_uuid(track) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Unassociated detections (custom): ";
      for (const auto & detection : detections_) {
        if (!custom_has_ass.has_association_detection(detection)) {
          ss << mot::get_uuid(detection) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Unassociated tracks (custom): ";
      for (const auto & track : track_manager_.get_all_tracks()) {
        if (!custom_has_ass.has_association_track(track)) {
          ss << mot::get_uuid(track) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    mot::HasAssociation has_ass{associations};

    {
      std::stringstream ss;
      ss << "Associated detections: ";
      for (const auto & detection : detections_) {
        if (has_ass(detection)) {
          ss << mot::get_uuid(detection) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Associated tracks: ";
      for (const auto & track : track_manager_.get_all_tracks()) {
        if (has_ass(track)) {
          ss << mot::get_uuid(track) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Unassociated detections: ";
      for (const auto & detection : detections_) {
        if (!has_ass(detection)) {
          ss << mot::get_uuid(detection) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    {
      std::stringstream ss;
      ss << "Unassociated tracks: ";
      for (const auto & track : track_manager_.get_all_tracks()) {
        if (!has_ass(track)) {
          ss << mot::get_uuid(track) << ' ';
        }
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    // Logging only
    {
      static constexpr mot::Visitor xyv_to_string = {
        [](mot::CtrvDetection const & detection) {
          std::stringstream ss;
          ss << "x: " << detection.state.position_x << ", ";
          ss << "y: " << detection.state.position_y << ", ";
          ss << "v: " << detection.state.velocity;

          return ss.str();
        },
        [](mot::CtraDetection const & detection) {
          std::stringstream ss;
          ss << "x: " << detection.state.position_x << ", ";
          ss << "y: " << detection.state.position_y << ", ";
          ss << "v: " << detection.state.velocity;

          return ss.str();
        }};

      static constexpr mot::Visitor track_xyv_to_string = {
        [](mot::CtrvTrack const & track) {
          std::stringstream ss;
          ss << "x: " << track.state.position_x << ", ";
          ss << "y: " << track.state.position_y << ", ";
          ss << "v: " << track.state.velocity;

          return ss.str();
        },
        [](mot::CtraTrack const & track) {
          std::stringstream ss;
          ss << "x: " << track.state.position_x << ", ";
          ss << "y: " << track.state.position_y << ", ";
          ss << "v: " << track.state.velocity;

          return ss.str();
        }};

      // std::stringstream ss;
      // ss << "Scores: \n";
      // for (const auto & [uuid_pair, score] : scores) {
      //   const auto result_a = std::find_if(
      //     std::cbegin(detections_), std::cend(detections_),
      //     [uuid = uuid_pair.first](const auto & o) { return mot::get_uuid(o) == uuid; });
      //   const auto ts = track_manager_.get_all_tracks();
      //   const auto result_b = std::find_if(
      //     std::cbegin(ts), std::cend(ts),
      //     [uuid = uuid_pair.second](const auto & o) { return mot::get_uuid(o) == uuid; });
      //   ss << uuid_pair.first << " (" << std::visit(xyv_to_string, *result_a) << "), "
      //      << uuid_pair.second << " (" << std::visit(track_xyv_to_string, *result_b) << "): " << score
      //      << '\n';
      // }
      // RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      std::stringstream ss;
      ss << "Scores: \n";
      for (const auto & [uuid_pair, score] : scores) {
        ss << uuid_pair.first << ", " << uuid_pair.second << ": " << score << '\n';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    // {
    //   std::stringstream ss;
    //   ss << "Mahalanobis scores: \n";
    //   for (const auto & [uuid_pair, score] : mahalanobis_scores) {
    //     ss << uuid_pair.first << ", " << uuid_pair.second << ": " << score << '\n';
    //   }
    //   RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    // }

    // {
    //   std::stringstream ss;
    //   ss << "Weighted scores: \n";
    //   for (const auto & [uuid_pair, score] : weighted_scores) {
    //     ss << uuid_pair.first << ", " << uuid_pair.second << ": " << score << '\n';
    //   }
    //   RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    // }

    track_manager_.update_track_lists(associations);

    // Logging only
    // {
    //   const auto result = std::find_if(
    //     std::cbegin(detections_), std::cend(detections_),
    //     [](const auto & d) { return mot::get_uuid(d) == mot::Uuid{"0002-207"}; });

    //   const auto tracks = track_manager_.get_all_tracks();
    //   const auto track_result = std::find_if(
    //     std::cbegin(tracks), std::cend(tracks),
    //     [](const auto & t) { return mot::get_uuid(t) == mot::Uuid{"0002-207"}; });

    //   if (result != std::cend(detections_) && track_result != std::cend(tracks)) {
    //     RCLCPP_ERROR_STREAM(
    //       this->get_logger(), "0002-207 (detection): " << std::visit(state_to_string, *result));
    //     RCLCPP_ERROR_STREAM(
    //       this->get_logger(),
    //       "0002-207 (track): " << std::visit(track_state_to_string, *track_result));
    //   }
    // }

    // Logging only
    {
      std::stringstream ss;
      ss << "Associations: \n";
      for (const auto & [track_uuid, detection_uuids] : associations) {
        ss << track_uuid << ": ";
        for (const auto & detection_uuid : detection_uuids) {
          ss << detection_uuid << ' ';
        }
        ss << '\n';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    std::unordered_map<mot::Uuid, Detection> detection_map;
    for (const auto & detection : detections_) {
      detection_map[mot::get_uuid(detection)] = detection;
    }

    const mot::HasAssociation has_association{associations};
    for (auto & track : track_manager_.get_all_tracks()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Track update (existing track): " << std::visit(track_state_to_string, track));
      if (has_association(track)) {
        RCLCPP_ERROR(get_logger(), "FOO");
        if (associations.count(get_uuid(track)) == 0) {
          RCLCPP_ERROR_STREAM(get_logger(), "Map has no key: " << get_uuid(track));
        }
        const auto detection_uuids{associations.at(get_uuid(track))};
        RCLCPP_ERROR(get_logger(), "BAR");
        const auto first_detection{detection_map[detection_uuids.at(0)]};
        RCLCPP_ERROR(get_logger(), "BAZ");
        RCLCPP_ERROR_STREAM(
          get_logger(), "Track update (detection)" << std::visit(state_to_string, first_detection));
        auto predicted_track{track};
        mot::propagate_to_time(
          predicted_track, current_time, mot::UnscentedTransform{1.0, 2.0, 0.0});
        // RCLCPP_ERROR_STREAM(
        //   get_logger(),
        //   "Track update (predicted track)" << std::visit(track_state_to_string, predicted_track));
        // const auto fused_track{
        //   std::visit(mot::covariance_intersection_visitor, predicted_track, first_detection)};
        const auto fused_track{
          std::visit(mot::covariance_intersection_visitor, track, first_detection)};
        // RCLCPP_ERROR_STREAM(
        //   get_logger(),
        //   "Track update (fused track)" << std::visit(track_state_to_string, fused_track));

        // mot::set_timestamp(track, mot::get_timestamp(fused_track));
        // mot::copy_state(track, fused_track);
        // mot::copy_state_covariance(track, fused_track);
        track_manager_.update_track(mot::get_uuid(track), fused_track);

        // track_manager_.update_track(
        //   mot::get_uuid(track), mot::get_timestamp(track), mot::get_state(track),
        //   mot::get_state_covariance(track));
      }
    }

    // Unassociated detections don't influence the tracking pipeline, so we can add
    // them to the tracker at the end.
    std::vector<Detection> unassociated_detections;
    for (const auto & [uuid, detection] : detection_map) {
      if (!has_association(detection)) {
        unassociated_detections.push_back(detection);
      }
    }

    {
      std::stringstream ss;
      ss << "Unassociated detections (pre filter): ";
      for (const auto & detection : unassociated_detections) {
        ss << mot::get_uuid(detection) << ' ';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    const auto is_lowest_score_within_threshold = [this, &scores](const auto & detection) {
      const auto uuid{mot::get_uuid(detection)};
      auto min_score{std::numeric_limits<float>::max()};
      for (const auto & [uuid_pair, score] : scores) {
        if (uuid_pair.second == uuid) {
          min_score = std::min(min_score, score);
        }
      }

      RCLCPP_ERROR_STREAM(this->get_logger(), "Min score (" << uuid << "): " << min_score);

      return min_score < 6.0;
    };

    auto remove_start = std::remove_if(
      std::begin(unassociated_detections), std::end(unassociated_detections),
      is_lowest_score_within_threshold);
    unassociated_detections.erase(remove_start, std::end(unassociated_detections));

    {
      std::stringstream ss;
      ss << "Unassociated detections (post filter): ";
      for (const auto & detection : unassociated_detections) {
        ss << mot::get_uuid(detection) << ' ';
      }
      RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
    }

    std::stringstream new_detections;
    new_detections << "New detections: ";
    const auto clusters{mot::cluster_detections(unassociated_detections, 0.75, CustomMetric{})};
    for (const auto & cluster : clusters) {
      const auto detection{std::cbegin(cluster.get_detections())->second};
      new_detections << mot::get_uuid(detection) << ' ';
      track_manager_.add_tentative_track(std::visit(make_track_visitor, detection));
    }
    new_detections << '\n';
    RCLCPP_ERROR(get_logger(), new_detections.str().c_str());

    std::stringstream confirmed_tracks;
    confirmed_tracks << "Confirmed tracks: ";
    carma_cooperative_perception_interfaces::msg::TrackList track_list;
    for (const auto & track : track_manager_.get_confirmed_tracks()) {
      confirmed_tracks << mot::get_uuid(track) << ' ';
      track_list.tracks.push_back(to_ros_msg(track));
    }
    confirmed_tracks << '\n';
    RCLCPP_ERROR(get_logger(), confirmed_tracks.str().c_str());

    track_list_pub_->publish(track_list);

    detections_.clear();
    uuid_index_map_.clear();

  } catch (std::out_of_range const & e) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "CAUGHT SOMETHING: std::out_of_range: " << e.what());
  } catch (std::exception const & e) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "CAUGHT SOMETHING: std::exception: " << e.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "CAUGHT SOMETHING: non std::exception");
  }
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::MultipleObjectTrackerNode)  // NOLINT
