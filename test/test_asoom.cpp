#include <gtest/gtest.h>
#include "asoom/pose_graph.h"

TEST(ASOOM_pose_graph_test, test_two_nodes) {
  auto pg = std::make_unique<PoseGraph>();
  ASSERT_TRUE(pg);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  // Make starting point not at 0,0,0 so optimizer does something
  pose.translate(Eigen::Vector3d(1,0,0)); 
  size_t ind = pg->addFrame(10, pose);
  EXPECT_EQ(ind, 0);

  pose.translate(Eigen::Vector3d(1,0,0)); 
  ind = pg->addFrame(20, pose);
  EXPECT_EQ(ind, 1);

  // Test optimization
  EXPECT_FLOAT_EQ(pg->getPoseAtIndex(0).translation()[0], 1);
  EXPECT_FLOAT_EQ(pg->getPoseAtIndex(1).translation()[0], 2);
  pg->update();
  EXPECT_FLOAT_EQ(pg->getPoseAtIndex(0).translation()[0], 0);
  EXPECT_FLOAT_EQ(pg->getPoseAtIndex(1).translation()[0], 1);

  // Test Time getter
  EXPECT_FALSE(pg->getPoseAtTime(100));
  EXPECT_TRUE(pg->getPoseAtTime(10));
  EXPECT_FLOAT_EQ(pg->getPoseAtTime(20)->translation()[0], 1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
