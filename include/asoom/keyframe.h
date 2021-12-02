#pragma once

#include <opencv2/core/core.hpp>
#include <memory>
#include <map>

/*!
 * Manage a single keyframe in the pose graph
 */
class Keyframe {
  public:
    Keyframe(long stamp, cv::Mat img) : stamp_(stamp), img_(img) {}

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    //! Timestamp in nsec from epoch
    const long stamp_;

    //! Image associated with keyframe
    cv::Mat img_;

    //! Depth image associated with keyframe for the same image
    cv::Mat depth_;
};

using Keyframes = std::map<long, std::shared_ptr<Keyframe>>;
