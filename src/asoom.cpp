#include <chrono>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "asoom/asoom.h"

ASOOM::ASOOM(const Params& asoom_params, const PoseGraph::Params& pg_params,
    const Rectifier::Params& rect_params, const DenseStereo::Params& stereo_params,
    const Map::Params& map_params) 
  : pose_graph_thread_(PoseGraphThread(this, pg_params)),
    stereo_thread_(StereoThread(this, rect_params, stereo_params)),
    map_thread_(MapThread(this, map_params)),
    params_(asoom_params) {}

ASOOM::~ASOOM() {
  // Set kill flag, then just wait for threads to complete
  exit_threads_flag_ = true;
  pose_graph_thread_.join();
  stereo_thread_.join();
  map_thread_.join();
}

void ASOOM::addFrame(long stamp, cv::Mat& img, const Eigen::Isometry3d& pose) {
  if (stamp > most_recent_stamp_) {
    most_recent_stamp_ = stamp;
  }

  std::scoped_lock<std::mutex> lock(keyframe_input_.m);
  keyframe_input_.buf.emplace_back(std::make_shared<Keyframe>(stamp, img, pose));
}

void ASOOM::addGPS(long stamp, const Eigen::Vector3d& pos) {
  std::scoped_lock<std::mutex> lock(gps_input_.m);
  gps_input_.buf.emplace_back(GPS(stamp, pos));
}

std::vector<Eigen::Isometry3d> ASOOM::getGraph() {
  std::vector<Eigen::Isometry3d> frame_vec;

  {
    std::shared_lock lock(keyframes_.m);
    for (const auto& frame : keyframes_.frames) {
      frame_vec.push_back(frame.second->getPose());
    }
  }

  return frame_vec;
}

Eigen::Array4Xf ASOOM::getDepthCloud(long stamp) {
  std::shared_lock lock(keyframes_.m);
  return keyframes_.frames.at(stamp)->getDepthCloud();
}

long ASOOM::getMostRecentStamp() const {
  // We could manage this inside the PoseGraph, but then we would
  // have to deal with thread-safety
  return most_recent_stamp_;
}

long ASOOM::getMostRecentStampWithDepth() {
  std::shared_lock lock(keyframes_.m);
  for (auto it=keyframes_.frames.rbegin(); it!=keyframes_.frames.rend(); it++) {
    if (it->second->hasDepth()) {
      return it->second->getStamp();
    }
  }
  return -1;
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

    double scale = pg_.getScale();
    double error = pg_.getError();
    // Do on one line, color red so don't interleave thread info
    std::cout << "\033[33m" << "[PGT] ====== Pose Graph Thread ======" << std::endl << 
      "[PGT] Buffer parsing: " << 
      duration_cast<microseconds>(parse_buffer_t - start_t).count() << "us" << std::endl <<
      "[PGT] GTSAM Optimization: " << 
      duration_cast<microseconds>(update_t - parse_buffer_t).count() << "us" << std::endl <<
      "[PGT] Keyframe Updating: " << 
      duration_cast<microseconds>(update_keyframes_t - update_t).count() << "us" << std::endl <<
      "[PGT] Scale: " << scale << std::endl <<
      "[PGT] Error: " << error << "\033[0m" << std::endl << std::flush;

    if (scale < 0) {
        std::cout << "\033[31m" << "[WARNING] Scale is negative" << "\033[0m" << std::endl;
    }

    next += milliseconds(asoom_->params_.pgo_thread_period_ms);
    std::this_thread::sleep_until(next);
  }
  std::cout << "\033[33m" << "[PGT] Pose Graph Thread Exited" << "\033[0m" << std::endl;
  return true;
}

void ASOOM::PoseGraphThread::parseBuffer() {
  {
    // VO Buffer
    std::scoped_lock<std::mutex> lock(asoom_->keyframe_input_.m);

    for (auto& frame : asoom_->keyframe_input_.buf) {
      size_t ind = pg_.addFrame(*frame);
      if ((pg_.getPoseAtIndex(ind).translation() - last_key_pos_).head<2>().norm() > 
          asoom_->params_.keyframe_dist_thresh_m && pg_.isInitialized()) {
        // Update pose from pg since it has adapted to scale and starting loc
        frame->setPose(pg_.getPoseAtIndex(ind));
        last_key_pos_ = frame->getPose().translation();

        // We have moved far enough, insert frame
        std::unique_lock lock(asoom_->keyframes_.m);
        // Use std::move since we are going to wipe buf anyway
        asoom_->keyframes_.frames.insert({frame->getStamp(), std::move(frame)});
      }
    }
    asoom_->keyframe_input_.buf.clear();
  }

  {
    // GPS Buffer
    std::scoped_lock<std::mutex> lock(asoom_->gps_input_.m);

    for (auto& gps : asoom_->gps_input_.buf) {
      pg_.addGPS(gps.stamp, std::move(gps.utm));
    }

    asoom_->gps_input_.buf.clear();
  }
}

void ASOOM::PoseGraphThread::updateKeyframes() {
  std::unique_lock lock(asoom_->keyframes_.m);

  for (auto& key : asoom_->keyframes_.frames) {
    auto new_pose = pg_.getPoseAtTime(key.first);
    if (new_pose) {
      key.second->setPose(*new_pose);
    }
  }
}

/***********************************************************
 * Stereo Thread
 ***********************************************************/

bool ASOOM::StereoThread::operator()() {
  if (!rectifier_.haveCalib()) {
    std::cout << "\033[34m" << "[StT] Running in no image mode" << "\033[0m" << std::endl;
    return true;
  }
  dense_stereo_.setIntrinsics(rectifier_.getOutputK(), rectifier_.getOutputSize());

  using namespace std::chrono;
  auto next = steady_clock::now();
  while (!asoom_->exit_threads_flag_) {
    auto start_t = high_resolution_clock::now();
    auto keyframes_to_compute = getKeyframesToCompute();
    auto buffer_keyframes_t = high_resolution_clock::now();
    computeDepths(keyframes_to_compute);
    auto compute_depths_t = high_resolution_clock::now();
    updateKeyframes(keyframes_to_compute);
    auto update_keyframes_t = high_resolution_clock::now();
    
    std::cout << "\033[34m" << "[StT] ======== Stereo Thread ========" << std::endl << 
      "[StT] Buffering Keyframes: " << 
      duration_cast<microseconds>(buffer_keyframes_t - start_t).count() << "us" << std::endl <<
      "[StT] Computing Depths: " << 
      duration_cast<microseconds>(compute_depths_t - buffer_keyframes_t).count() << "us" << std::endl <<
      "[StT] Keyframe Updating: " << 
      duration_cast<microseconds>(update_keyframes_t - compute_depths_t).count() << "us" << std::endl <<
      "[StT] Total Keyframes Updated: " << std::max<int>(0, keyframes_to_compute.size() - 1) << 
      std::endl << "\033[0m" << std::flush;

    next += milliseconds(asoom_->params_.stereo_thread_period_ms);
    std::this_thread::sleep_until(next);
  }
  std::cout << "\033[34m" << "[StT] Stereo Thread Exited" << "\033[0m" << std::endl;
  return true;
}

std::vector<Keyframe> ASOOM::StereoThread::getKeyframesToCompute() {
  std::shared_lock lock(asoom_->keyframes_.m);

  std::vector<Keyframe> keyframes_to_compute;
  std::shared_ptr<Keyframe> last_keyframe;
  for (const auto& frame : asoom_->keyframes_.frames) {
    if (!frame.second->hasDepth()) {
      if (last_keyframe) {
        if (last_keyframe->hasDepth()) {
          // If the previous keyframe has depth then it has not already been added
          keyframes_to_compute.push_back(*last_keyframe);
        }
      }
      // When we do this we call Keyframe's copy constructor
      // Notably the image copies (cv::Mat) are not deep.  Makes this more efficient,
      // but need to be careful for thread safety
      keyframes_to_compute.push_back(*frame.second);
    }
    last_keyframe = frame.second;
  }

  return keyframes_to_compute;
}

void ASOOM::StereoThread::computeDepths(std::vector<Keyframe>& frames) {
  // Use c ptr because we don't want the pointer to try to manage the underlying memory
  const Keyframe *last_frame = nullptr;
  cv::Mat i1m1, i1m2, i2m1, i2m2, rect1, rect2, disp;
  for (auto& frame : frames) {
    if (last_frame != nullptr) {
      auto new_dposes = rectifier_.genRectifyMaps(frame, *last_frame, i1m1, i1m2, i2m1, i2m2);
      // Rectify images
      rectifier_.rectifyImage(frame.getImg(), i1m1, i1m2, rect1);
      rectifier_.rectifyImage(last_frame->getImg(), i2m1, i2m2, rect2);

      // Do stereo
      dense_stereo_.computeDisp(rect1, rect2, disp);

      double baseline = 
        (frame.getPose().translation() - last_frame->getPose().translation()).norm();
      frame.setDepth(new_dposes.first, rect1, dense_stereo_.projectDepth(disp, baseline));
    }
    last_frame = &frame;
  }
}

void ASOOM::StereoThread::updateKeyframes(const std::vector<Keyframe>& frames) {
  std::unique_lock lock(asoom_->keyframes_.m);
  for (const auto& frame : frames) {
    if (!asoom_->keyframes_.frames.at(frame.getStamp())->hasDepth()) {
      asoom_->keyframes_.frames.at(frame.getStamp())->setDepth(frame);
    }
  }
}

/***********************************************************
 * Map Thread
 ***********************************************************/

bool ASOOM::MapThread::operator()() {
  using namespace std::chrono;
  auto next = steady_clock::now();
  while (!asoom_->exit_threads_flag_) {
    std::cout << "\033[35m" << "[Map] ========== Map Thread =========" << std::endl << 
      "\033[0m" << std::flush;

    next += milliseconds(asoom_->params_.map_thread_period_ms);
    std::this_thread::sleep_until(next);
  }
  std::cout << "\033[35m" << "[Map] Map Thread Exited" << "\033[0m" << std::endl;
  return true;
}
