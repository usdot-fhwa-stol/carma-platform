// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/filter.h>


#include <socketcan_interface/string.h>
#include <socketcan_interface/dummy.h>

// Bring in gtest
#include <gtest/gtest.h>


TEST(FilterTest, simpleMask)
{
  const std::string msg1("123#");
  const std::string msg2("124#");

  can::FrameFilter::Ptr f1 = can::tofilter("123");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
}

TEST(FilterTest, maskTests)
{
  const std::string msg1("123#");
  const std::string msg2("124#");
  const std::string msg3("122#");

  can::FrameFilter::Ptr f1 = can::tofilter("123:123");
  can::FrameFilter::Ptr f2 = can::tofilter("123:ffe");
  can::FrameFilter::Ptr f3 = can::tofilter("123~123");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
  EXPECT_FALSE(f1->pass(can::toframe(msg3)));


  EXPECT_TRUE(f2->pass(can::toframe(msg1)));
  EXPECT_FALSE(f2->pass(can::toframe(msg2)));
  EXPECT_TRUE(f2->pass(can::toframe(msg3)));

  EXPECT_FALSE(f3->pass(can::toframe(msg1)));
  EXPECT_TRUE(f3->pass(can::toframe(msg2)));
  EXPECT_TRUE(f3->pass(can::toframe(msg3)));

}

TEST(FilterTest, rangeTest)
{
  const std::string msg1("120#");
  const std::string msg2("125#");
  const std::string msg3("130#");

  can::FrameFilter::Ptr f1 = can::tofilter("120-120");
  can::FrameFilter::Ptr f2 = can::tofilter("120_120");
  can::FrameFilter::Ptr f3 = can::tofilter("120-125");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
  EXPECT_FALSE(f1->pass(can::toframe(msg3)));

  EXPECT_FALSE(f2->pass(can::toframe(msg1)));
  EXPECT_TRUE(f2->pass(can::toframe(msg2)));
  EXPECT_TRUE(f2->pass(can::toframe(msg3)));

  EXPECT_TRUE(f3->pass(can::toframe(msg1)));
  EXPECT_TRUE(f3->pass(can::toframe(msg2)));
  EXPECT_FALSE(f3->pass(can::toframe(msg3)));

}

class Counter {
public:
    size_t count_;
    Counter(): count_(0) {}
    void count(const can::Frame &frame) {
      ++count_;
    }
};

TEST(FilterTest, listenerTest)
{

  Counter counter;
  boost::shared_ptr<can::CommInterface> dummy(new can::DummyInterface(true));

  can::FilteredFrameListener::FilterVector filters;
  filters.push_back(can::tofilter("123:FFE"));

  can::CommInterface::FrameListener::Ptr  listener(new can::FilteredFrameListener(dummy,can::CommInterface::FrameDelegate(&counter, &Counter::count), filters));

  can::Frame f1 = can::toframe("123#");
  can::Frame f2 = can::toframe("124#");
  can::Frame f3 = can::toframe("122#");

  dummy->send(f1);
  EXPECT_EQ(1, counter.count_);
  dummy->send(f2);
  EXPECT_EQ(1, counter.count_);
  dummy->send(f3);
  EXPECT_EQ(2, counter.count_);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
