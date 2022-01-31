#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <memory>
#include <map>
#include <iostream>

namespace Eigen {
  using Array5Xd = Array<double, 5, Dynamic>;
  using Array5Xf = Array<float, 5, Dynamic>;
  using Array6Xd = Array<double, 6, Dynamic>;
}

using DepthCloudArray = Eigen::Array5Xf;

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
      return have_depth_;
    }

    inline bool hasSem() const {
      return !sem_img_.empty();
    }

    inline const cv::Mat& getSem() const {
      return sem_img_;
    }

    inline bool hasRepublished() const {
      return republished_;
    }

    inline void republish() {
      republished_ = true;
    }

    inline void setPose(const Eigen::Isometry3d& p) {
      pose_ = p;
    }

    inline void setDepth(const Eigen::Isometry3d& dp, const cv::Mat& rect_img,
        const std::shared_ptr<Eigen::Array3Xd>& depth) {
      rect_dpose_ = dp;
      rect_img_ = rect_img;
      depth_ = depth;
      if (depth_) {
        have_depth_ = true;
      }
      on_disk_ = false;
    }

    inline void setDepth(const Keyframe& key) {
      setDepth(key.rect_dpose_, key.rect_img_, key.depth_);
    }

    inline void setSem(const cv::Mat& sem) {
      if (sem.type() == CV_8UC1) {
        sem_img_ = sem;
        on_disk_ = false;
      }
    }

    DepthCloudArray getDepthCloud() const;

    bool needsMapUpdate() const;

    inline bool inMap() const {
      return !map_pose_.matrix().isIdentity(1e-5);
    }

    inline void updateMapPose() {
      map_pose_ = pose_;
    }

    //! Saves data to disk, then wipes data
    void saveToDisk();

    //! Returns true if load successful
    bool loadFromDisk();

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    //! Timestamp in nsec from epoch
    const long stamp_;

    //! Current global pose estimation
    Eigen::Isometry3d pose_;

    /*! 
     * The pose used by the mapper.  If pose_ changes from this significantly, we
     * want to trigger a map rebuild
     */
    Eigen::Isometry3d map_pose_{Eigen::Isometry3d::Identity()};

    //! Corrective rotation for rectification
    Eigen::Isometry3d rect_dpose_{Eigen::Isometry3d::Identity()};

    // These cv::Mats and shared_ptr allow Keyframes to be copied quickly, at the cost 
    // of not being a deep copy.  However, this is ok, since these are never modified
    // in keyframes_ directly once they are set.

    //! Image associated with keyframe
    cv::Mat img_;
    cv::Mat rect_img_;

    //! Semantic image, later can be overwritten with rect image
    cv::Mat sem_img_;

    bool have_depth_ = false;
    //! Depth cloud associated with keyframe for the same image, stored row-major
    std::shared_ptr<Eigen::Array3Xd> depth_;

    //! True if the keyframe image has been republished
    bool republished_ = false;

    //! True if data on disk is up to date
    bool on_disk_ = false;

    //! True if data currently in memory
    bool in_mem_ = true;

    /***********************************************************
     * LOCAL STATIC FUNCTIONS
     ***********************************************************/

    static void saveDataBinary(const cv::Mat& img, std::ofstream& outfile);
    static void saveDataBinary(const std::shared_ptr<Eigen::Array3Xd>& arr, 
        std::ofstream& outfile);

    static void readDataBinary(std::ifstream& infile, cv::Mat& img);
    static void readDataBinary(std::ifstream& infile, 
        std::shared_ptr<Eigen::Array3Xd>& arr);
};

// Using pointers here should make sort faster
using Keyframes = std::map<long, std::unique_ptr<Keyframe>>;
