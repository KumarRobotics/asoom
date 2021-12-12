#pragma once

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "asoom/keyframe.h"

class Rectifier {
  public:
    /*!
     * Exception class to indicate calib we can't handle
     */
    class invalid_camera_exception: public std::exception
    {
      virtual const char* what() const throw()
      {
        return "Invalid camera calib file";
      }
    };

    Rectifier(const std::string& calib_path);

    /*!
     * We could just modify in-situ, but this would require a write thread lock on 
     * the keyframes for the whole time.  Also we avoid returning the new projection 
     * matrices because all are the same and stored in this class
     *
     * @param key1 First keyframe
     * @param key2 Second keyframe
     * @param rect1_map1 Map 1 for first image
     * @param rect1_map2 Map 2 for first image
     * @param rect2_map1 Map 1 for second image
     * @param rect2_map2 Map 2 for second image
     * @return Pair of relative rotation transforms
     */
    std::pair<Eigen::Isometry3d, Eigen::Isometry3d> genRectifyMaps(const Keyframe& key1,
        const Keyframe& key2, cv::Mat& rect1_map1, cv::Mat& rect1_map2,
        cv::Mat& rect2_map1, cv::Mat& rect2_map2);

    /*!
     * Really just a wrapper for cv::remap.  Separate from generating the rect maps
     * beacuse we need to also apply the same map to the semantic layer
     */
    void rectifyImage(const cv::Mat& input, const cv::Mat& map1, const cv::Mat& map2, 
        cv::Mat& output);

    /*!
     * @return Instrinsic K matrix for rectified images
     */
    Eigen::Matrix3d getOutputK() const;

  private:
    //! Intrinsics and dist coeff for input camera
    cv::Mat input_K_, input_dist_;
    cv::Size input_size_;

    //! Intrinsics for rectified images
    cv::Mat output_K_;
};
