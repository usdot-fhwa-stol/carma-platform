#include "message_to_external_object_convertor.h"
#include <carma_v2x_msgs/psm.hpp>
#include <carma_perception_msgs/external_object.hpp>

namespace object
{

    class BsmToExternalObject : public MessageToExternalObjectConvertor<carma_v2x_msgs::msg::BSM>
    {
        void convert(const carma_v2x_msgs::msg::BSM &in_msg, carma_perception_msgs::msg::ExternalObject &out_msg)
        {            
            // TODO
            
        }
    };

}