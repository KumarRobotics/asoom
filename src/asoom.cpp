#include "asoom/asoom.h"

ASOOM::ASOOM(const PoseGraph::Params& pg_params) {
  pg_ = std::make_shared<PoseGraph>(pg_params);
  keyframes_ = std::make_shared<Keyframes>();
}

void ASOOM::addFrame(long stamp, cv::Mat& img, const Eigen::Isometry3d& pose) {
  keyframes_->emplace(stamp, std::make_shared<Keyframe>(stamp, img));
  pg_->addFrame(stamp, pose);
}
