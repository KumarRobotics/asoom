// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(ASOOM_test, test_test) {
  ASSERT_TRUE(true);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
