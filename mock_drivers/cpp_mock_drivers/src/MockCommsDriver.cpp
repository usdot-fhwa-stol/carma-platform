#include "cpp_mock_drivers/MockDriver.h"
#include "cpp_mock_drivers/MockDriverNode.h"
// #include <carma_utils/CARMAUtils.h>
#include <cav_msgs/ByteArray.h>

namespace mock_drivers{

    class MockCommsDriver : public MockDriver{

            private:

                ROSComms<cav_msgs::ByteArray> inbound_pub_;
                ROSComms<cav_msgs::ByteArray, const cav_msgs::ByteArray::ConstPtr&> outbound_pub_;

                void outbound_callback(const cav_msgs::ByteArray::ConstPtr& msg){

                };

            public:

            MockCommsDriver(){
                MockDriverNode mock_driver_node_();

                // mock_drivers::ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&> test_comms(fcnPtr, ct, false, 10, "ooga_booga");

                ROSComms<cav_msgs::ByteArray> inbound_pub_(CommTypes::pub, false, 10, "inbound_binary_msg");
                
                // std::function<void(const std_msgs::String::ConstPtr&)> fcnPtr = std::bind(&TestCB::test_call, &tcb, std::placeholders::_1);
                std::function<void(const cav_msgs::ByteArray::ConstPtr&)> outbound_ptr = std::bind(&MockCommsDriver::outbound_callback, this, std::placeholders::_1);
                ROSComms<cav_msgs::ByteArray, const cav_msgs::ByteArray::ConstPtr&> outbound_pub_(outbound_ptr, CommTypes::sub, false, 10, "outbound_binary_msg");

                // mock_driver_node_.addComms() (x2)
            }

    };

}