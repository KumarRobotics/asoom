#include <iostream>
#include "asoom/keyframe.h"

Eigen::Array4Xf Keyframe::getDepthCloud() const {
  if (!hasDepth()) {
    return Eigen::Array4Xf::Zero(4, 0);
  }
  // xyz, transformed to global frame
  Eigen::Isometry3d trans = getRectPose();
  Eigen::Array3Xd transformed_pos = (trans.linear() * depth_->matrix()).rowwise() + 
    trans.translation().transpose();

  Eigen::Array4Xf cloud(4, depth_->cols());
  size_t dense_ind;
  size_t sparse_ind = 0;
  cv::Vec3b bgr;
  for (size_t x=0; x<rect_img_.size().width; x++) {
    for (size_t y=0; y<rect_img_.size().height; y++) {
      // Get row major index
      dense_ind = y*rect_img_.size().width + x;

      // Undefined depth
      if (std::isinf((*depth_)(2, sparse_ind))) {
        continue;
      }
      
      // The weird PCL color format
      bgr = rect_img_.at<cv::Vec3b>(y, x);
      std::uint32_t rgb_packed = (static_cast<uint32_t>(bgr[2]) << 16 | 
                                  static_cast<uint32_t>(bgr[1]) << 8 | 
                                  static_cast<uint32_t>(bgr[0]));
      cloud.col(sparse_ind) << transformed_pos.col(dense_ind).cast<float>(), 
        *reinterpret_cast<float*>(&rgb_packed);
      sparse_ind++;
    }
  }

  return cloud;
}
