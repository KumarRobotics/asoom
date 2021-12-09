#include <chrono>

#include "asoom/asoom.h"

ASOOM::ASOOM(const Params& asoom_params, const PoseGraph::Params& pg_params) 
  : pose_graph_thread_(PoseGraphThread(this, pg_params)),
    params_(asoom_params) {}

ASOOM::~ASOOM() {
  // Set kill flag, then just wait for threads to complete
  exit_threads_flag_ = true;
  pose_graph_thread_.join();
}

void ASOOM::addFrame(long stamp, cv::Mat& img, const Eigen::Isometry3d& pose) {
  if (stamp > most_recent_stamp_) {
    most_recent_stamp_ = stamp;
  }

  std::scoped_lock<std::mutex>(keyframe_input_.m);
  keyframe_input_.buf.emplace_back(std::make_shared<Keyframe>(stamp, img, pose));
}

void ASOOM::addGPS(long stamp, const Eigen::Vector3d& pos) {
  std::scoped_lock<std::mutex>(gps_input_.m);
  gps_input_.buf.emplace_back(GPS(stamp, pos));
}

std::vector<Eigen::Isometry3d> ASOOM::getGraph() {
  std::vector<Eigen::Isometry3d> frame_vec;

  {
    std::scoped_lock<std::mutex>(keyframes_.m);
    for (const auto& frame : keyframes_.frames) {
      frame_vec.push_back(frame.second->getPose());
    }
  }

  return frame_vec;
}

long ASOOM::getMostRecentStamp() const {
  // We could manage this inside the PoseGraph, but then we would
  // have to deal with thread-safety
  return most_recent_stamp_;
}

/***********************************************************
 * PoseGraph Thread
 ***********************************************************/

bool ASOOM::PoseGraphThread::operator()() {
  // Initialize to something far away from origin
  last_key_pos_ = Eigen::Vector3d::Constant(3, 1, -10000);

  using namespace std::chrono;
  auto next = steady_clock::now();
  while (!asoom_->exit_threads_flag_) {
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

    next += milliseconds(asoom_->params_.pgo_thread_period_ms);
    std::this_thread::sleep_until(next);
  }
  std::cout << "\033[31m" << "[PGT] Pose Graph Thread Exited" << "\033[0m" << std::endl;
  return true;
}

void ASOOM::PoseGraphThread::parseBuffer() {
  {
    // VO Buffer
    std::scoped_lock<std::mutex>(asoom_->keyframe_input_.m);

    for (auto& frame : asoom_->keyframe_input_.buf) {
      size_t ind = pg_.addFrame(*frame);
      if ((pg_.getPoseAtIndex(ind).translation() - last_key_pos_).head<2>().norm() > 
          asoom_->params_.keyframe_dist_thresh_m && pg_.isInitialized()) {
        // Update pose from pg since it has adapted to scale and starting loc
        frame->setPose(pg_.getPoseAtIndex(ind));
        last_key_pos_ = frame->getPose().translation();

        // We have moved far enough, insert frame
        std::scoped_lock<std::mutex>(asoom_->keyframes_.m);
        // Use std::move since we are going to wipe buf anyway
        asoom_->keyframes_.frames.insert({frame->getStamp(), std::move(frame)});
      }
    }
    asoom_->keyframe_input_.buf.clear();
  }

  {
    // GPS Buffer
    std::scoped_lock<std::mutex>(asoom_->gps_input_.m);

    for (auto& gps : asoom_->gps_input_.buf) {
      pg_.addGPS(gps.stamp, std::move(gps.utm));
    }

    asoom_->gps_input_.buf.clear();
  }
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
