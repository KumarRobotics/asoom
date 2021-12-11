#include "asoom/rectifier.h"

Rectifier::Rectifier(const std::string& calib_path) {
}

std::pair<Eigen::Isometry3d, Eigen::Isometry3d> Rectifier::rectify(const Keyframe& key1,
    const Keyframe& key2, cv::Mat& im1_rect, cv::Mat& im2_rect) {
  return std::make_pair(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
}

Eigen::Matrix3d Rectifier::getOutputK() const {
  Eigen::Matrix3d K_eig;
  cv::cv2eigen(output_K_, K_eig);
  return K_eig;
}
