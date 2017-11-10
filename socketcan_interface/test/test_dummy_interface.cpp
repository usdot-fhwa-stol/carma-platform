// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/dummy.h>

// Bring in gtest
#include <gtest/gtest.h>

class DummyInterfaceTest : public ::testing::Test{
public:
    std::list<std::string> responses;
    can::DummyInterface dummy;
    DummyInterfaceTest() : dummy(true), listener(dummy.createMsgListener(can::CommInterface::FrameDelegate(this, &DummyInterfaceTest::handle ))) { }

   void handle(const can::Frame &f){
        responses.push_back(can::tostring(f, true));
    }
    can::CommInterface::FrameListener::Ptr listener;
};

// Declare a test
TEST_F(DummyInterfaceTest, testCase1)
{
    dummy.add("0#8200", "701#00" ,false);

    std::list<std::string> expected;

    dummy.send(can::toframe("0#8200"));
    expected.push_back("0#8200");
    expected.push_back("701#00");

    EXPECT_EQ(expected, responses);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}