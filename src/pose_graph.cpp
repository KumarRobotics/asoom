#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "asoom/pose_graph.h"

PoseGraph::PoseGraph() : size_(0), initial_pose_factor_id_(-1) {
  graph_ = std::make_unique<gtsam::NonlinearFactorGraph>();
}

size_t PoseGraph::addFrame(long stamp, const Eigen::Isometry3d& pose) {
  if (size_ > 0) {
    // Get different from most recent
    auto most_recent_pose = pose_history_.rbegin()->second;
    auto diff = most_recent_pose->pose.inverse() * pose;
    graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(most_recent_pose->key, P(size_), Eigen2GTSAM(diff),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones(6, 1)));
  } else {
    // Create prior on first pose to remove free degree of freedom until GPS installed
    initial_pose_factor_id_ = graph_->size();
    graph_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(P(size_), Eigen2GTSAM(Eigen::Isometry3d::Identity()),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Zero()));
  }
  pose_history_.emplace(stamp, std::make_shared<OriginalPose>(pose, P(size_)));
  current_opt_.insert(P(size_), Eigen2GTSAM(pose));
  size_++;
  return size_ - 1;
}

void PoseGraph::addGPS(long stamp, const Eigen::Vector3d& utm_pose) {
}

void PoseGraph::update() {
  gtsam::LevenbergMarquardtParams opt_params;
  gtsam::LevenbergMarquardtOptimizer opt(*graph_, current_opt_, opt_params);
  current_opt_ = opt.optimize();
}

std::optional<Eigen::Isometry3d> PoseGraph::getPoseAtTime(long stamp) const {
  auto element = pose_history_.find(stamp);
  if (element == pose_history_.end()) return {};

  return GTSAM2Eigen(current_opt_.at<gtsam::Pose3>(element->second->key));
}

Eigen::Isometry3d PoseGraph::getPoseAtIndex(size_t ind) const {
  if (ind >= size_) {
    throw std::out_of_range("Index out of range");
  }
  return GTSAM2Eigen(current_opt_.at<gtsam::Pose3>(P(ind)));
}

double PoseGraph::getScale() const {
  return 1;
}

size_t PoseGraph::size() const {
  return size_;
}
