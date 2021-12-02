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
  using namespace std::chrono;
  auto next = steady_clock::now();
  while (true) {
    auto start_t = high_resolution_clock::now();
    parseBuffer();
    auto parse_buffer_t = high_resolution_clock::now();
    pg_.update();
    auto update_t = high_resolution_clock::now();
    updateKeyframes();
    auto update_keyframes_t = high_resolution_clock::now();

    // Do on one line, color red so don't interleave thread info
    std::cout << "\033[31m" << "[PGT] ====== Pose Graph Thread ======" << std::endl << 
      "[PGT] Buffer parsing: " << 
      duration_cast<microseconds>(parse_buffer_t - start_t).count() << "us" << std::endl <<
      "[PGT] GTSAM Optimization: " << 
      duration_cast<microseconds>(update_t - parse_buffer_t).count() << "us" << std::endl <<
      "[PGT] Keyframe Updating: " << 
      duration_cast<microseconds>(update_keyframes_t - update_t).count() << "us" << 
      "\033[0m" << std::endl;

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
