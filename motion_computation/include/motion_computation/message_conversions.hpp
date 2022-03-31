#include <carma_v2x_msgs/psm.hpp>
#include <carma_perception_msgs/external_object.hpp>

namespace object
{

namespace conversion
{
  void convert(
    const carma_v2x_msgs::msg::PSM &in_msg, 
    carma_perception_msgs::msg::ExternalObject &out_msg,
    const std::string& map_frame_id, 
    double pred_period, 
    double pred_step_size,
    const lanelet::projection::LocalFrameProjector& map_projector,
    tf2::Quaternion ned_in_map_rotation
  );

  void convert(const carma_v2x_msgs::msg::BSM &in_msg, carma_perception_msgs::msg::ExternalObject &out_msg, 
      const std::string& map_frame_id, double pred_period, double pred_step_size);
}
}