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
#include <limits>
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
    [](const auto &) {
      // Currently on support CTRV and CTRA
      throw std::runtime_error{"cannot make ROS 2 message from track type"};
    }};

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

  declare_parameter(
    "execution_frequency_hz", mot::remove_units(units::frequency::hertz_t{1 / execution_period_}));

  declare_parameter(
    "track_promotion_threshold", static_cast<int>(track_manager_.get_promotion_threshold().value));

  declare_parameter(
    "track_removal_threshold", static_cast<int>(track_manager_.get_promotion_threshold().value));

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
    mot::propagate_to_time(detection, end_time, mot::default_unscented_transform);
  }
}

static auto predict_track_states(std::vector<Track> tracks, units::time::second_t end_time)
{
  for (auto & track : tracks) {
    mot::propagate_to_time(track, end_time, mot::default_unscented_transform);
  }

  return tracks;
}

/**
 * @brief Calculate 2D Euclidean distance between track and detection
 *
 * If both the track and detection semantic classes are known, this function
 * object returns the 2D Euclidean distance between their states. Otherwise,
 * it returns std::nullopt.
*/
struct SemanticDistance2dScore
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      const auto dist{two_dimensional_distance(track.state, detection.state)};

      // Fall back to 2D Euclidean distance if either semantic class if unknown. The unknown
      // track/detection may actually be the same other track/detection we are scoring against.
      if (
        track.semantic_class == mot::SemanticClass::kUnknown ||
        detection.semantic_class == mot::SemanticClass::kUnknown) {
        return dist;
      }

      if (track.semantic_class == detection.semantic_class) {
        return dist;
      }
    }

    return std::nullopt;
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & t, const auto & d) { return (*this)(t, d); }, track, detection);
  }

private:
  static auto two_dimensional_distance(const mot::CtrvState & lhs, const mot::CtrvState & rhs)
    -> float
  {
    const auto x_diff_sq{
      std::pow(mot::remove_units(lhs.position_x) - mot::remove_units(rhs.position_x), 2)};
    const auto y_diff_sq{
      std::pow(mot::remove_units(lhs.position_y) - mot::remove_units(rhs.position_y), 2)};

    return std::sqrt(x_diff_sq + y_diff_sq);
  }

  static auto two_dimensional_distance(const mot::CtraState & lhs, const mot::CtraState & rhs)
    -> float
  {
    const auto x_diff_sq{
      std::pow(mot::remove_units(lhs.position_x) - mot::remove_units(rhs.position_x), 2)};
    const auto y_diff_sq{
      std::pow(mot::remove_units(lhs.position_y) - mot::remove_units(rhs.position_y), 2)};

    return std::sqrt(x_diff_sq + y_diff_sq);
  }
};

/**
 * @brief Calculates distance between a point and detection in SE(2) (special Euclidean) space
*/
struct MetricSe2
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
      [](const mot::Point &, const auto &) {
        // Currently on support CTRV and CTRA
        return std::numeric_limits<double>::max();
      }};

    return std::visit(visitor, std::variant<mot::Point>{point}, detection);
  }
};

auto MultipleObjectTrackerNode::execute_pipeline() -> void
{
  static constexpr mot::Visitor make_track_visitor{
    [](const mot::CtrvDetection & d, const mot::Uuid & u) {
      return Track{mot::make_track<mot::CtrvTrack>(d, u)};
    },
    [](const mot::CtraDetection & d, const mot::Uuid & u) {
      return Track{mot::make_track<mot::CtraTrack>(d, u)};
    },
    [](const auto &) {
      // Currently on support CTRV and CTRA
      throw std::runtime_error("cannot make track from given detection");
    },
  };

  if (track_manager_.get_all_tracks().empty()) {
    RCLCPP_DEBUG(
      get_logger(), "List of tracks is empty. Converting detections to tentative tracks");

    // This clustering distance is an arbitrarily-chosen heuristic. It is working well for our
    // current purposes, but there's no reason it couldn't be restricted or loosened.
    const auto clusters{mot::cluster_detections(detections_, 0.75)};
    for (const auto & cluster : clusters) {
      const auto detection{std::cbegin(cluster.get_detections())->second};
      const auto uuid_str{mot::get_uuid(detection).value()};
      // CARLA uses three-digit actor identifiers. We want to UUID scheme to be
      // <track_number>-<carla_actor_id> for easier visual analysis by users.
      const mot::Uuid new_uuid{
        std::to_string(lifetime_generated_track_count_++) +
        uuid_str.substr(std::size(uuid_str) - 4, 4)};
      track_manager_.add_tentative_track(
        std::visit(make_track_visitor, detection, std::variant<mot::Uuid>(new_uuid)));
    }

    track_list_pub_->publish(carma_cooperative_perception_interfaces::msg::TrackList{});

    detections_.clear();
    uuid_index_map_.clear();
    return;
  }

  const units::time::second_t current_time{this->now().seconds()};

  temporally_align_detections(detections_, current_time);

  const auto predicted_tracks{predict_track_states(track_manager_.get_all_tracks(), current_time)};
  auto scores{
    mot::score_tracks_and_detections(predicted_tracks, detections_, SemanticDistance2dScore{})};

  // This pruning distance is an arbitrarily-chosen heuristic. It is working well for our
  // current purposes, but there's no reason it couldn't be restricted or loosened.
  mot::prune_track_and_detection_scores_if(scores, [](const auto & score) { return score > 5.0; });

  const auto associations{
    mot::associate_detections_to_tracks(scores, mot::gnn_association_visitor)};

  track_manager_.update_track_lists(associations);

  std::unordered_map<mot::Uuid, Detection> detection_map;
  for (const auto & detection : detections_) {
    detection_map[mot::get_uuid(detection)] = detection;
  }

  const mot::HasAssociation has_association{associations};
  for (auto & track : track_manager_.get_all_tracks()) {
    if (has_association(track)) {
      const auto detection_uuids{associations.at(get_uuid(track))};
      const auto first_detection{detection_map[detection_uuids.at(0)]};
      const auto fused_track{
        std::visit(mot::covariance_intersection_visitor, track, first_detection)};
      track_manager_.update_track(mot::get_uuid(track), fused_track);
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

  // We want to remove unassociated tracks that are close enough to existing tracks
  // to avoid creating duplicates. Duplicate tracks will cause association inconsistencies
  // (flip flopping associations between the two tracks).
  auto remove_start{std::remove_if(
    std::begin(unassociated_detections), std::end(unassociated_detections),
    [&scores](const auto & detection) {
      const auto uuid{mot::get_uuid(detection)};
      auto min_score{std::numeric_limits<float>::infinity()};
      for (const auto & [uuid_pair, score] : scores) {
        if (uuid_pair.second == uuid) {
          min_score = std::min(min_score, score);
        }
      }

      // This distance is an arbitrarily-chosen heuristic. It is working well for our
      // current purposes, but there's no reason it couldn't be restricted or loosened.
      return min_score < 1.0;
    })};

  unassociated_detections.erase(remove_start, std::end(unassociated_detections));

  // This clustering distance is an arbitrarily-chosen heuristic. It is working well for our
  // current purposes, but there's no reason it couldn't be restricted or loosened.
  const auto clusters{mot::cluster_detections(unassociated_detections, 0.75, MetricSe2{})};
  for (const auto & cluster : clusters) {
    const auto detection{std::cbegin(cluster.get_detections())->second};
    const auto uuid_str{mot::get_uuid(detection).value()};
    // CARLA uses three-digit actor identifiers. We want to UUID scheme to be
    // <track_number>-<carla_actor_id> for easier visual analysis by users.
    const mot::Uuid new_uuid{
      std::to_string(lifetime_generated_track_count_++) +
      uuid_str.substr(std::size(uuid_str) - 4, 4)};
    track_manager_.add_tentative_track(
      std::visit(make_track_visitor, detection, std::variant<mot::Uuid>(new_uuid)));
  }

  carma_cooperative_perception_interfaces::msg::TrackList track_list;
  for (const auto & track : track_manager_.get_confirmed_tracks()) {
    track_list.tracks.push_back(to_ros_msg(track));
  }

  track_list_pub_->publish(track_list);

  detections_.clear();
  uuid_index_map_.clear();
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::MultipleObjectTrackerNode)  // NOLINT
