// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/string.h>

// Bring in gtest
#include <gtest/gtest.h>


TEST(StringTest, stringconversion)
{
  const std::string s1("123#1234567812345678");
  can::Frame f1 = can::toframe(s1);
  EXPECT_EQ(s1, can::tostring(f1, true));

  const std::string s2("1337#1234567812345678");
  can::Frame f2 = can::toframe(s2);
  EXPECT_FALSE(f2.isValid());

  const std::string s3("80001337#1234567812345678");
  can::Frame f3 = can::toframe(s3);
  EXPECT_EQ(s3, can::tostring(f3, true));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
