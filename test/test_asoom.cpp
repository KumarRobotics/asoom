#include <gtest/gtest.h>
#include "asoom/asoom.h"

TEST(ASOOM_asoom_test, test_pgo_thread) {
  ASOOM a(ASOOM::Params(100, 1.5), PoseGraph::Params(0.1, 0.1, 0.1, 0, true));

  // Sanity check
  EXPECT_EQ(a.getGraph().size(), 0);

  // Build simple graph
  cv::Mat img; // Just empty image since we aren't doing anything with it
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(1,0,0)); 
  a.addFrame(10, img, pose);
  pose.translate(Eigen::Vector3d(1,0,0)); 
  pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
  a.addFrame(20, img, pose);
  pose.translate(Eigen::Vector3d(0,-1,0)); 
  pose.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX()));
  a.addFrame(30, img, pose);

  // Wait for optimizer to run
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto poses = a.getGraph();

  // Verify that nothing has really changed, except translation to 0
  // We don't expect the middle point, since didn't move enough
  ASSERT_EQ(poses.size(), 2);
  EXPECT_FLOAT_EQ(poses[0].translation()[0], 0);
  EXPECT_FLOAT_EQ(poses[1].translation()[0], 2);
}

TEST(ASOOM_asoom_test, test_pgo_gps_thread) {
  ASOOM a(ASOOM::Params(100, 0), PoseGraph::Params(0.1, 0.1, 0.1, 0, false, 2));

  // Sanity check
  EXPECT_EQ(a.getGraph().size(), 0);

  // Build simple graph
  cv::Mat img; // Just empty image since we aren't doing anything with it
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(1,0,0)); 
  a.addFrame(10, img, pose);
  pose.translate(Eigen::Vector3d(1,0,0)); 
  pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
  a.addFrame(20, img, pose);

  a.addGPS(10, Eigen::Vector3d(10,0,0));
  a.addGPS(20, Eigen::Vector3d(20,0,0));

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  // Haven't initialized yet
  EXPECT_EQ(a.getGraph().size(), 0);

  pose.translate(Eigen::Vector3d(0,-1,0)); 
  pose.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX()));
  a.addFrame(30, img, pose);
  a.addGPS(30, Eigen::Vector3d(30,0,0));

  // Wait for optimizer to run
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto poses = a.getGraph();

  // Verify that nothing has really changed, except translation to 0
  // We don't expect the middle point, since didn't move enough
  ASSERT_EQ(poses.size(), 1);
  EXPECT_FLOAT_EQ(poses[0].translation()[0], 30);
}
