#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <memory>
#include <map>

namespace Eigen {
  using Array5Xd = Array<double, 5, Dynamic>;
  using Array6Xd = Array<double, 6, Dynamic>;
}

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

    inline Eigen::Isometry3d getRectPose() const {
      return pose_ * rect_dpose_;
    }

    inline cv::Mat getImg() const {
      return img_;
    }

    inline bool hasDepth() const {
      return depth_.cols() > 0;
    }

    inline void setPose(const Eigen::Isometry3d& p) {
      pose_ = p;
    }

    inline void setDepth(const Eigen::Isometry3d& dp, const cv::Mat& rect_img,
        const Eigen::Array3Xd& depth) {
      rect_dpose_ = dp;
      rect_img_ = rect_img;
      depth_ = depth;
    }

    inline void setDepth(const Keyframe& key) {
      rect_dpose_ = key.rect_dpose_;
      rect_img_ = key.rect_img_;
      depth_ = key.depth_;
    }

    Eigen::Array4Xd getDepthCloud() const;

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    //! Timestamp in nsec from epoch
    const long stamp_;

    //! Current global pose estimation
    Eigen::Isometry3d pose_;

    //! Corrective rotation for rectification
    Eigen::Isometry3d rect_dpose_{Eigen::Isometry3d::Identity()};

    //! Image associated with keyframe
    const cv::Mat img_;
    cv::Mat rect_img_;

    //! Depth cloud associated with keyframe for the same image, stored row-major
    Eigen::Array3Xd depth_{Eigen::Array3Xd::Zero(3, 0)};
};

// Using pointers here should make sort faster
using Keyframes = std::map<long, std::shared_ptr<Keyframe>>;
