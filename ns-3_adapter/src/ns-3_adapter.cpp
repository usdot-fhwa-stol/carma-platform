#include "ns-3_adapter.h"

void NS3Adapter::initialize()
{
    //Comms Subscriber
    comms_sub_ = comms_api_nh_->subscribe("outbound_binary_msg", queue_size_, &NS3Adapter::onOutboundMessage, this);
}