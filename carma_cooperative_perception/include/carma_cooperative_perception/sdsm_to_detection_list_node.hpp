#ifndef CARMA_COOPERATIVE_PERCEPTION_SDSM_TO_DETECTION_NODE_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_SDSM_TO_DETECTION_NODE_HPP_

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_ros2_utils/msg_conversion_node.hpp>
#include <j3224_v2x_msgs/msg/sensor_data_sharing_message.hpp>
#include <rclcpp/rclcpp.hpp>

namespace carma_cooperative_perception
{

inline auto to_detection_list_msg(const j3224_v2x_msgs::msg::SensorDataSharingMessage & msg)
  -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  return carma_cooperative_perception_interfaces::msg::DetectionList{};
}

class SdsmToDetectionListNode : public carma_ros2_utils::CarmaLifecycleNode
{
  using input_msg_type = j3224_v2x_msgs::msg::SensorDataSharingMessage;
  using input_msg_shared_pointer = typename input_msg_type::SharedPtr;
  using output_msg_type = carma_cooperative_perception_interfaces::msg::DetectionList;

public:
  explicit SdsmToDetectionListNode(const rclcpp::NodeOptions & options)
  : CarmaLifecycleNode{options},
    publisher_{create_publisher<output_msg_type>("output/detections", 1)},
    subscription_{create_subscription<input_msg_type>(
      "input/sdsm", 1, [this](input_msg_shared_pointer msg_ptr) { sdsm_msg_callback(*msg_ptr); })}
  {
  }

  auto sdsm_msg_callback(const input_msg_type & msg) const noexcept -> void
  {
    publisher_->publish(to_detection_list_msg(msg));
  }

private:
  rclcpp::Publisher<output_msg_type>::SharedPtr publisher_;
  rclcpp::Subscription<input_msg_type>::SharedPtr subscription_;
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_SDSM_TO_DETECTION_NODE_HPP_
