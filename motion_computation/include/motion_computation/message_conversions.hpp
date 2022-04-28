#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_v2x_msgs/msg/mobility_path.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

namespace motion_computation {

namespace conversion {

void convert(const carma_v2x_msgs::msg::PSM& in_msg, carma_perception_msgs::msg::ExternalObject& out_msg,
             const std::string& map_frame_id, double pred_period, double pred_step_size,
             const lanelet::projection::LocalFrameProjector& map_projector, const tf2::Quaternion& ned_in_map_rotation, 
             rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock);

void convert(const carma_v2x_msgs::msg::BSM& in_msg, carma_perception_msgs::msg::ExternalObject& out_msg,
             const std::string& map_frame_id, double pred_period, double pred_step_size,
             const lanelet::projection::LocalFrameProjector& map_projector, tf2::Quaternion ned_in_map_rotation);

void convert(const carma_v2x_msgs::msg::MobilityPath &in_msg, carma_perception_msgs::msg::ExternalObject &out_msg,
             const lanelet::projection::LocalFrameProjector &map_projector);
}  // namespace conversion
}  // namespace motion_computation