#include <gtest/gtest.h>
#include <ros/package.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "asoom/dense_stereo.h"
#include "asoom/rectifier.h"

TEST(ASOOM_dense_stereo_test, test_stereo) {
  /*
  2 Keyframes:
  1635642164881558848:
 [-33.9279
   30.7013
   6.56765]
 [-0.462416
   0.886253
  -0.0204316
   0.0176213] //xyzw
  1635642165797544512:
 [-34.7402
   31.3199
   6.54712]
 [-0.670737
   0.731146
   0.0986779
   0.0761593] //xyzw
   */
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
  Keyframe k1(1635642164881558848, im1, pose1);
  
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
  Keyframe k2(1635642165797544512, im2, pose2);

  Rectifier rect(Rectifier::Params(
        ros::package::getPath("asoom") + "/config/grace_quarters.yaml", 0.5));
  cv::Mat i1m1, i1m2, i2m1, i2m2;
  auto transforms = rect.genRectifyMaps(k1, k2, i1m1, i1m2, i2m1, i2m2);

  cv::Mat rect1, rect2;
  rect.rectifyImage(im1, i1m1, i1m2, rect1);
  rect.rectifyImage(im2, i2m1, i2m2, rect2);

  DenseStereo stereo(DenseStereo::Params{});
  cv::Mat disp;
  stereo.computeDisp(rect1, rect2, disp);
  cv::imwrite("asoom_disp_viz.png", disp*255/80);
  std::cout << "Wrote asoom_disp_viz.png to test disparity" << std::endl << std::flush;

  // Approximate depth at center of image
  // d = (fx * baseline) / disp
  double depth_center = ((pose1.translation() - pose2.translation()).norm() * 
      rect.getOutputK()(0, 0)) / disp.at<double>(disp.size().height/2, disp.size().width/2);
  EXPECT_NEAR(depth_center, 30, 5);
  // Should be 0 everywhere where depth undefined
  EXPECT_FLOAT_EQ(disp.at<double>(10, disp.size().width-1), 0);

  // Have not set intrinsics yet
  EXPECT_THROW({
      stereo.projectDepth(disp, 1);
    }, DenseStereo::intrinsic_mismatch_exception
  );
  stereo.setIntrinsics(rect.getOutputK(), rect.getOutputSize());

  std::shared_ptr<Eigen::Array3Xd> depth_pc = stereo.projectDepth(disp, 
      (pose1.translation() - pose2.translation()).norm());
  EXPECT_FLOAT_EQ((*depth_pc)(2, disp.size().height*disp.size().width/2 + disp.size().width/2), 
      depth_center);
}
