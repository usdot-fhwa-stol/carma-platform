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
#include <cooperative_perception/clustering.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/fusing.hpp>
#include <cooperative_perception/scoring.hpp>
#include <cooperative_perception/temporal_alignment.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace carma_cooperative_perception
{

namespace mot = cooperative_perception;

auto make_ctrv_detection(
  const carma_cooperative_perception_interfaces::msg::Detection & msg) noexcept -> Detection
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

  const mot::CtrvStateCovariance covariance = mot::CtrvStateCovariance::Zero();

  return mot::CtrvDetection{timestamp, state, covariance, mot::Uuid{msg.id}};
}

auto make_ctra_detection(
  const carma_cooperative_perception_interfaces::msg::Detection & msg) noexcept -> Detection
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

  const mot::CtraStateCovariance covariance = mot::CtraStateCovariance::Zero();

  return mot::CtraDetection{timestamp, state, covariance, mot::Uuid{msg.id}};
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
      break;
  }

  throw std::runtime_error("unkown motion model type '" + std::to_string(msg.motion_model) + "'");
}

static auto to_ros_msg(const mot::CtraTrack & track) noexcept
{
  carma_cooperative_perception_interfaces::msg::Track msg;

  msg.header.stamp.sec = mot::remove_units(units::math::floor(track.timestamp));
  msg.header.stamp.nanosec = mot::remove_units(
    units::time::nanosecond_t{units::math::fmod(track.timestamp, units::time::second_t{10.0})});
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

  return msg;
}

static auto to_ros_msg(const mot::CtrvTrack & track) noexcept
{
  carma_cooperative_perception_interfaces::msg::Track msg;

  msg.header.stamp.sec = mot::remove_units(units::math::floor(track.timestamp));
  msg.header.stamp.nanosec = mot::remove_units(
    units::time::nanosecond_t{units::math::fmod(track.timestamp, units::time::second_t{10.0})});
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
  pipeline_execution_timer_ = create_wall_timer(period_ns, [this] { execute_pipeline(); });

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
  const carma_cooperative_perception_interfaces::msg::DetectionList & msg) noexcept -> void
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
  std::vector<Detection> & detections, units::time::second_t end_time) noexcept -> void
{
  for (auto & detection : detections) {
    mot::propagate_to_time(detection, end_time, mot::default_unscented_transform);
  }
}

static auto predict_track_states(std::vector<Track> tracks, units::time::second_t end_time) noexcept
{
  for (auto & track : tracks) {
    mot::propagate_to_time(track, end_time, mot::default_unscented_transform);
  }

  return tracks;
}

auto MultipleObjectTrackerNode::execute_pipeline() -> void
{
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

  temporally_align_detections(detections_, current_time);

  const auto predicted_tracks{predict_track_states(track_manager_.get_all_tracks(), current_time)};
  const auto scores{
    mot::score_tracks_and_detections(predicted_tracks, detections_, mot::euclidean_distance_score)};

  const auto associations{mot::associate_detections_to_tracks(scores, mot::gnn_associator)};
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
      track = std::visit(mot::covariance_intersection_visitor, track, first_detection);
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

  const auto clusters{mot::cluster_detections(unassociated_detections, 0.75)};
  for (const auto & cluster : clusters) {
    const auto detection{std::cbegin(cluster.get_detections())->second};
    track_manager_.add_tentative_track(std::visit(make_track_visitor, detection));
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
