#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <memory>
#include <map>

/*!
 * Manage a single keyframe in the pose graph
 */
class Keyframe {
  public:
    Keyframe(long stamp, cv::Mat img, const Eigen::Isometry3d& pose) 
      : stamp_(stamp), img_(img), pose_(pose) {}

    // Setters and getters
    inline long getStamp() const {
      return stamp_;
    }

    inline Eigen::Isometry3d getPose() const {
      return pose_;
    }

    inline void setPose(const Eigen::Isometry3d& p) {
      pose_ = p;
    }

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    //! Timestamp in nsec from epoch
    const long stamp_;

    //! Current global pose estimation
    Eigen::Isometry3d pose_;

    //! Image associated with keyframe
    cv::Mat img_;

    //! Depth image associated with keyframe for the same image
    cv::Mat depth_;
};

using Keyframes = std::map<long, std::shared_ptr<Keyframe>>;
