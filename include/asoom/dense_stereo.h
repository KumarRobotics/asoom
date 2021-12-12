#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "asoom/keyframe.h"

class DenseStereo {
  public:
    struct Params {
      // These are the settings for sgbm
      // See https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html
      int min_disparity;
      int num_disparities;
      int block_size;
      int P1_coeff; // Multiplied by num_channels * block_size^2
      int P2_coeff;
      int disp_12_map_diff;
      int pre_filter_cap;
      int uniqueness_ratio;
      int speckle_window_size;
      int speckle_range;

      Params(int md, int nd, int bs, int p1c, int p2c, int d12md, int pfc, int ur,
          int sws, int sr) : 
        min_disparity(md), num_disparities(nd), block_size(bs), P1_coeff(p1c), P2_coeff(p2c), 
        disp_12_map_diff(d12md), pre_filter_cap(pfc), uniqueness_ratio(ur), 
        speckle_window_size(sws), speckle_range(sr) {}

      Params() : Params(1, 80, 9, 1, 3, 0, 35, 10, 100, 20) {}
    };

    DenseStereo(const Params& params);

    /*!
     * @param im1 First keyframe to compute depth of
     * @param im2 Second keyframe to compute depth of
     * @param disp Output disparity
     */
    void computeDepth(const cv::Mat& im1, const cv::Mat& im2, cv::Mat& disp);

  private:
    cv::Ptr<cv::StereoSGBM> stereo_;
};
