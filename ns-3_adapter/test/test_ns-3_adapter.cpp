#include <ns-3_adapter.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(NS3AdapterTest, testOnConnectHandler)
{
    int argc = 1;
    //char* argv[] {c[0], c[1]};
    char **argv;
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << static_cast<int>(worker.getDriverStatus().status));
    worker.onConnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OPERATIONAL);
}

TEST(NS3AdapterTest, testOnDisconnectHandler)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << static_cast<int>(worker.getDriverStatus().status));
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

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << static_cast<int>(worker.getDriverStatus().status));
    EXPECT_THROW(worker.onMessageReceivedHandler(content, id), ros::TimeNotInitializedException); //Since the onMessageReceivedHandler requires ros::Time initialized, this should throw an exception
    
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);


}

TEST(NS3AdapterTest, testpackMessage)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray array1;

    uint8_t msg = 4;

    array1.content.push_back(msg);

    
    auto pm = worker.packMessage(array1);

    ASSERT_GT(pm.size(), 0);
    ASSERT_EQ(pm.size(), 189);


}

TEST(NS3AdapterTest, testonOutboundMessage)
{
    /*int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray array1;

    uint8_t msg = 1;

    array1.content.push_back(msg);

    cav_msgs::ByteArrayPtr message;
    ROS_ERROR_STREAM("THISISATEST");
    //message->content.push_back(msg);
    worker.onOutboundMessage(message);
    auto msg_q = worker.getMsgQueue();
    EXPECT_EQ(msg_q.size(), 0);*/

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

TEST(NS3AdapterTest, testcompose_handshake_msg)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);
    std::string result = worker.compose_handshake_msg("default_id", "ego1", "2000", "127.0.0.1");
    std::cout << result << std::endl;
}
