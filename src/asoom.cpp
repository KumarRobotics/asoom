#include <chrono>

#include "asoom/asoom.h"

ASOOM::ASOOM(const PoseGraph::Params& pg_params) 
  : pose_graph_thread_(PoseGraphThread(this, pg_params)) {}

void ASOOM::addFrame(long stamp, cv::Mat& img, const Eigen::Isometry3d& pose) {
  std::scoped_lock<std::mutex>(keyframe_input_.m);
  keyframe_input_.buf.emplace_back(std::make_shared<Keyframe>(stamp, img, pose));
}

/***********************************************************
 * PoseGraph Thread
 ***********************************************************/

bool ASOOM::PoseGraphThread::operator()() {
  auto next = std::chrono::steady_clock::now();
  while (true) {
    std::cout << "Pose Graph Thread" << std::endl;

    parseBuffer();
    pg_.update();
    updateKeyframes();

    next += 1000ms;
    std::this_thread::sleep_until(next);
  }
  return true;
}

void ASOOM::PoseGraphThread::parseBuffer() {
  std::scoped_lock<std::mutex>(asoom_->keyframe_input_.m);

  for (auto& frame : asoom_->keyframe_input_.buf) {
    size_t ind = pg_.addFrame(*frame);
    if ((pg_.getPoseAtIndex(ind).translation() - last_key_pos_).head<2>().norm() > 5
        && pg_.isInitialized()) {
      last_key_pos_ = pg_.getPoseAtIndex(ind).translation();

      // We have moved far enough, insert frame
      std::scoped_lock<std::mutex>(asoom_->keyframes_.m);
      asoom_->keyframes_.frames.insert({frame->getStamp(), frame});
    }
  }
  asoom_->keyframe_input_.buf.clear();
}

void ASOOM::PoseGraphThread::updateKeyframes() {
  std::scoped_lock<std::mutex>(asoom_->keyframes_.m);

  for (auto& key : asoom_->keyframes_.frames) {
    auto new_pose = pg_.getPoseAtTime(key.first);
    if (new_pose) {
      key.second->setPose(*new_pose);
    }
  }
}
