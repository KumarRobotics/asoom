#include <gtest/gtest.h>
#include <ros/package.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "asoom/rectifier.h"

TEST(ASOOM_rectifier_test, test_init) {
  EXPECT_THROW({
      Rectifier rect("/no/config/here");
    }, YAML::BadFile
  );

  Rectifier rect(ros::package::getPath("asoom") + "/config/grace_quarters.yaml");
  auto K = rect.getOutputK();

  // Sanity check the matrix
  EXPECT_FLOAT_EQ(K(1, 0), 0);
  EXPECT_FLOAT_EQ(K(0, 1), 0);
  EXPECT_FLOAT_EQ(K(2, 0), 0);
  EXPECT_FLOAT_EQ(K(2, 1), 0);
  EXPECT_FLOAT_EQ(K(2, 2), 1);
  EXPECT_TRUE(K(0, 0) > 100);
  EXPECT_TRUE(K(1, 1) > 100);
  EXPECT_TRUE(K(0, 2) > 100);
  EXPECT_TRUE(K(1, 2) > 100);
}

TEST(ASOOM_rectifier_test, test_rect) {
  /*
  2 Keyframes:
  1635642164881558848:
 [-33.937
   30.7682
   6.58341]
 [-0.452344
   0.891474
  -0.0170222
   0.0192312]
  1635642168151356224:
 [-34.6666
   25.8773
   6.51482]
  [0.995337
   0.0289028
   0.090143
   0.018527]
   */
  cv::Mat im1 = cv::imread(ros::package::getPath("asoom") + 
                           "/test/test_imgs/1635642164881558848.jpg");
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translate(Eigen::Vector3d(
       -33.937,
        30.7682,
        6.58341));
  pose1.rotate(Eigen::Quaterniond(
       -0.452344,
        0.891474,
       -0.0170222,
        0.0192312));
  Keyframe k1(1635642164881558848, im1, pose1);
  
  cv::Mat im2 = cv::imread(ros::package::getPath("asoom") + 
                           "/test/test_imgs/1635642168151356224.jpg");
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translate(Eigen::Vector3d(
       -34.6666,
        25.8773,
        6.51482));
  pose2.rotate(Eigen::Quaterniond(
        0.995337,
        0.0289028,
        0.090143,
        0.018527));
  Keyframe k2(1635642168151356224, im2, pose2);

  Rectifier rect(ros::package::getPath("asoom") + "/config/grace_quarters.yaml");
  cv::Mat i1m1, i1m2, i2m1, i2m2;
  auto transforms = rect.genRectifyMaps(k1, k2, i1m1, i1m2, i2m1, i2m2);

  // Each rotation should take each pose to a common frame
  EXPECT_TRUE(Eigen::Quaterniond((pose1 * transforms.first).rotation()).angularDistance(
              Eigen::Quaterniond((pose2 * transforms.second).rotation())) < 0.01);
}
