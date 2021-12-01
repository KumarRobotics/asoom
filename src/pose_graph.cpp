#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "asoom/pose_graph.h"
#include "asoom/between_pose_scale_factor.h"

PoseGraph::PoseGraph() : size_(0), initial_pose_factor_id_(-1) {
  graph_ = std::make_unique<gtsam::NonlinearFactorGraph>();
  current_opt_.insert(S(0), 1.0);
  // Create prior on scale initially because otherwise unconstrained
  initial_scale_factor_id_ = graph_->size();
  graph_->emplace_shared<gtsam::PriorFactor<double>>(S(0), 1.0,
      gtsam::noiseModel::Constrained::All(1));
}

size_t PoseGraph::addFrame(long stamp, const Eigen::Isometry3d& pose) {
  if (size_ > 0) {
    // Get different from most recent
    auto most_recent_pose = pose_history_.rbegin()->second;
    auto diff = most_recent_pose->pose.inverse() * pose;
    graph_->emplace_shared<gtsam::BetweenPoseScaleFactor>(most_recent_pose->key, P(size_), S(0), Eigen2GTSAM(diff),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones(6, 1)*0.1));
    // Use current optimization estimates to improve initial guess
    diff.translation() *= getScale();
    auto most_recent_pose_opt = GTSAM2Eigen(current_opt_.at<gtsam::Pose3>(most_recent_pose->key));
    current_opt_.insert(P(size_), Eigen2GTSAM(most_recent_pose_opt * diff));
  } else {
    // Create prior on first pose to remove free degree of freedom until GPS installed
    initial_pose_factor_id_ = graph_->size();
    graph_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(P(size_), Eigen2GTSAM(Eigen::Isometry3d::Identity()),
        gtsam::noiseModel::Constrained::All(6));
    current_opt_.insert(P(size_), Eigen2GTSAM(Eigen::Isometry3d::Identity()));
  }
  pose_history_.emplace(stamp, std::make_shared<OriginalPose>(pose, P(size_)));
  size_++;
  return size_ - 1;
}

void PoseGraph::addGPS(long stamp, const Eigen::Vector3d& utm_pose) {
  if (graph_->exists(initial_pose_factor_id_)) {
    graph_->remove(initial_pose_factor_id_);
  }
  if (graph_->exists(initial_scale_factor_id_)) {
    graph_->remove(initial_scale_factor_id_);
    // Perturb scale to make optimizer not get stuck
    current_opt_.update(S(0), getScale() + 0.001);
  }

  if (pose_history_.find(stamp) == pose_history_.end()) {
    // GPS does not align, need to interpolate
  } else {
    // GPS stamp aligns perfectly, great!
    auto key = pose_history_.find(stamp)->second->key;
    graph_->emplace_shared<gtsam::GPSFactor>(key, utm_pose,
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Ones(6, 1)*0.1));
  }
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
  return current_opt_.at<double>(S(0));
}

size_t PoseGraph::size() const {
  return size_;
}

double PoseGraph::getError() const {
  return graph_->error(current_opt_);
}
