#include <iostream>
#include "asoom/keyframe.h"
#include "asoom/semantic_color_lut.h"

DepthCloudArray Keyframe::getDepthCloud() const {
  if (!hasDepth()) {
    return DepthCloudArray::Zero(5, 0);
  }

  // transform to global frame
  Eigen::Isometry3d trans = getRectPose();

  DepthCloudArray cloud(5, depth_->cols());
  size_t dense_ind;
  size_t sparse_ind = 0;
  cv::Vec3b bgr;
  for (size_t x=0; x<rect_img_.size().width; x++) {
    for (size_t y=0; y<rect_img_.size().height; y++) {
      // Get row major index
      dense_ind = y*rect_img_.size().width + x;

      // Undefined depth
      if ((*depth_)(2, dense_ind) > 80 || (*depth_)(2, dense_ind) < 1) {
        continue;
      }
      
      // The weird PCL color format
      bgr = rect_img_.at<cv::Vec3b>(y, x);
      uint32_t rgb_packed = SemanticColorLut::packColor(bgr[2], bgr[1], bgr[0]);
      cloud.block<3,1>(0, sparse_ind) = (trans * depth_->col(dense_ind)).cast<float>();
      cloud(3, sparse_ind) = *reinterpret_cast<float*>(&rgb_packed);

      // We pack in the class integer in the same way
      uint32_t sem_class = 0;
      if (hasSem()) {
        sem_class = sem_img_.at<uint8_t>(y, x);
      }
      cloud(4, sparse_ind) = *reinterpret_cast<float*>(&sem_class); 

      sparse_ind++;
    }
  }

  cloud.conservativeResize(Eigen::NoChange_t(), sparse_ind);
  return cloud;
}

bool Keyframe::needsMapUpdate() const {
  Eigen::Isometry3d diff = pose_.inverse() * map_pose_;
  if (diff.translation().norm() > 1 || 
      Eigen::AngleAxisd(diff.rotation()).angle() > 5*M_PI/180) {
    return true;
  }
  return false;
}
