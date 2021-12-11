#pragma once

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "asoom/keyframe.h"

class Rectifier {
  public:
    Rectifier(const std::string& calib_path);

    /*!
     * We could just modify in-situ, but this would require a write thread lock on 
     * the keyframes for the whole time.  Also we avoid returning the new projection 
     * matrices because all are the same and stored in this class
     *
     * @param key1 First keyframe
     * @param key2 Second keyframe
     * @param im1_rect Rectified key1, output ref
     * @param im2_rect Rectified key1, output ref
     * @return Pair of relative rotation transforms
     */
    std::pair<Eigen::Isometry3d, Eigen::Isometry3d> rectify(const Keyframe& key1,
        const Keyframe& key2, cv::Mat& im1_rect, cv::Mat& im2_rect);

    /*!
     * @return Instrinsic K matrix for rectified images
     */
    Eigen::Matrix3d getOutputK() const;

  private:
    //! Intrinsics and dist coeff for input camera
    cv::Mat input_K_, input_dist_;

    //! Intrinsics for rectified images
    cv::Mat output_K_;
};
