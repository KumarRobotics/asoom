#include <gtest/gtest.h>
#include <ros/package.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "asoom/asoom.h"

TEST(ASOOM_asoom_test, test_pgo_thread) {
  ASOOM a(ASOOM::Params(100, 100, 100, 1.5), PoseGraph::Params(0.1, 0.1, 0.1, 0, true),
      Rectifier::Params(), DenseStereo::Params(), Map::Params());

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
  EXPECT_FLOAT_EQ(poses[1].translation()[0], -2);
}

TEST(ASOOM_asoom_test, test_pgo_gps_thread) {
  ASOOM a(ASOOM::Params(100, 100, 100, 0), PoseGraph::Params(0.1, 0.1, 0.1, 0, false, 2),
      Rectifier::Params(), DenseStereo::Params(), Map::Params());

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

  // Now we have initialized, make sure next pose is in the right place
  ASSERT_EQ(poses.size(), 1);
  EXPECT_FLOAT_EQ(poses[0].translation()[0], 30);
}

TEST(ASOOM_asoom_test, test_stereo_thread) {
  ASOOM a(ASOOM::Params(100, 100, 100, 0.1), PoseGraph::Params(0.1, 0.1, 0.1, 0, true),
    Rectifier::Params(ros::package::getPath("asoom") + "/config/grace_quarters.yaml", 0.5), 
    DenseStereo::Params(), Map::Params());

  cv::Mat im1 = cv::imread(ros::package::getPath("asoom") + 
                           "/test/test_imgs/1635642164881558848.jpg");
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translate(Eigen::Vector3d(
       -33.9279,
        30.7013,
        6.56765));
  pose1.rotate(Eigen::Quaterniond(
        0.0176213,
       -0.462416,
        0.886253,
       -0.0204316)); //wxyz
  a.addFrame(0, im1, pose1);
  
  cv::Mat im2 = cv::imread(ros::package::getPath("asoom") + 
                           "/test/test_imgs/1635642165797544512.jpg");
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translate(Eigen::Vector3d(
       -34.7402,
        31.3199,
        6.54712));
  pose2.rotate(Eigen::Quaterniond(
        0.0761593,
       -0.670737,
        0.731146,
        0.0986779)); //wxyz
  a.addFrame(100, im2, pose2);

  // Wait long enough that stereo has completed
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  Eigen::Array4Xf pc = a.getDepthCloud(0);
  EXPECT_EQ(pc.rows(), 4);
  EXPECT_EQ(pc.cols(), 0);

  pc = a.getDepthCloud(100);
  EXPECT_EQ(pc.rows(), 4);
  EXPECT_TRUE(pc.cols() > 10000);
}
