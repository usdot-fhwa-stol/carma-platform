#include <ns-3_adapter.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(NS3AdapterTest, testOnConnectHandler)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    //char* argv[] {c[0], c[1]};
    char **argv;
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << worker.getDriverStatus().status);
    worker.onConnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OPERATIONAL);
}

TEST(NS3AdapterTest, testOnDisconnectHandler)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << worker.getDriverStatus().status);
    worker.onDisconnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);

}

TEST(NS3AdapterTest, testOnMsgReceivedHandler)
{
   int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);
    uint16_t id = 123;
    std::vector<uint8_t> content;
    content.push_back(1);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << worker.getDriverStatus().status);
    EXPECT_THROW(worker.onMessageReceivedHandler(content, id), ros::TimeNotInitializedException);
    
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);


}

TEST(NS3AdapterTest, testpackMessage)
{
    
}

TEST(NS3AdapterTest, testonOutboundMessage)
{
    /*int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray array1;

    uint8_t msg = 8;

    array1.content.push_back(msg);

    cav_msgs::ByteArray::ConstPtr message = new cav_msgs::ByteArray::ConstPtr(array1);


    worker.onOutboundMessage(message);
    EXPECT_EQ(worker.getMsgQueue().back(), message->content.back());*/

}

TEST(NS3AdapterTest, testSendMessageSrv)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_srvs::SendMessage::Request req;
    cav_srvs::SendMessage::Response resp;

    uint8_t msg = 8;
    req.message_to_send.content.push_back(msg);

   bool result = worker.sendMessageSrv(req, resp);

   EXPECT_EQ(result, true);
   EXPECT_EQ(resp.errorStatus, 1);

}
