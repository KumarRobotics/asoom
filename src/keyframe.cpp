#include "asoom/keyframe.h"

Eigen::Array4Xd Keyframe::getDepthCloud() const {
  // xyz, transformed to global frame
  Eigen::Isometry3d trans = getRectPose();
  Eigen::Array3Xd transformed_pos = (trans.linear() * depth_.matrix()).rowwise() + 
    trans.translation().transpose();

  Eigen::Array4Xd cloud(4, depth_.cols());
  size_t ind;
  for (size_t x=0; x<rect_img_.size().width; x++) {
    for (size_t y=0; y<rect_img_.size().height; y++) {
      // Get row major index
      ind = y*rect_img_.size().width + x;
    }
  }

  return cloud;
}
