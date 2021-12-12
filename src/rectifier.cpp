#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "asoom/rectifier.h"

Rectifier::Rectifier(const std::string& calib_path) {
  YAML::Node calib = YAML::LoadFile(calib_path);
  if (calib["cam0"]["camera_model"].as<std::string>() != "pinhole" || 
      calib["cam0"]["distortion_model"].as<std::string>() != "radtan" ||
      calib["cam0"]["distortion_coeffs"].size() < 4 ||
      calib["cam0"]["intrinsics"].size() != 4 ||
      calib["cam0"]["resolution"].size() != 2) {
    throw invalid_camera_exception();
  }

  input_K_ = cv::Mat::eye(3, 3, CV_64F);
  input_dist_ = cv::Mat::zeros(4, 1, CV_64F);

  input_K_.at<double>(0, 0) = calib["cam0"]["intrinsics"][0].as<double>();
  input_K_.at<double>(1, 1) = calib["cam0"]["intrinsics"][1].as<double>();
  input_K_.at<double>(0, 2) = calib["cam0"]["intrinsics"][2].as<double>();
  input_K_.at<double>(1, 2) = calib["cam0"]["intrinsics"][3].as<double>();

  for (size_t ind=0; ind<4; ind++) {
    input_dist_.at<double>(ind, 0) = calib["cam0"]["distortion_coeffs"][ind].as<double>();
  }

  input_size_.width = calib["cam0"]["resolution"][0].as<int>();
  input_size_.height = calib["cam0"]["resolution"][1].as<int>();

  // Now figure out what K should be for rectified images
  output_K_ = cv::getOptimalNewCameraMatrix(input_K_, input_dist_, input_size_, 0);
}

std::pair<Eigen::Isometry3d, Eigen::Isometry3d> Rectifier::genRectifyMaps(
    const Keyframe& key1, const Keyframe& key2, cv::Mat& rect1_map1, cv::Mat& rect1_map2,
    cv::Mat& rect2_map1, cv::Mat& rect2_map2)
{
  // Based on "A compact algorithm for rectification of stereo pairs",
  // Fusiello, Trucco, Verri, Machine Vision and Applications 2000
  
  auto R1 = key1.getPose().rotation();
  auto R2 = key2.getPose().rotation();
  auto T1 = key1.getPose().translation();
  auto T2 = key2.getPose().translation();

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R.row(0) = (T1 - T2).normalized();
  R.row(1) = (R1.row(2).cross(R.row(0))).normalized();
  R.row(2) = (R.row(0).cross(R.row(1))).normalized();

  return std::make_pair(Eigen::Isometry3d(R1.inverse() * R), 
                        Eigen::Isometry3d(R2.inverse() * R));
}

void Rectifier::rectifyImage(const cv::Mat& input, const cv::Mat& map1, const cv::Mat& map2, 
    cv::Mat& output)
{
  cv::remap(input, output, map1, map2, cv::INTER_LINEAR);
}

Eigen::Matrix3d Rectifier::getOutputK() const {
  Eigen::Matrix3d K_eig;
  cv::cv2eigen(output_K_, K_eig);
  return K_eig;
}
