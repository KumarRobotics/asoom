#include <iostream>
#include "asoom/dense_stereo.h"

DenseStereo::DenseStereo(const Params& params) {
  int P1 = params.P1_coeff*3*params.block_size*params.block_size;
  int P2 = params.P2_coeff*3*params.block_size*params.block_size;

  stereo_ = cv::StereoSGBM::create(params.min_disparity, params.num_disparities,
      params.block_size, P1, P2, params.disp_12_map_diff, params.pre_filter_cap,
      params.uniqueness_ratio, params.speckle_window_size, params.speckle_range);
}

void DenseStereo::computeDepth(const cv::Mat& im1, const cv::Mat& im2, cv::Mat& disp) {
  stereo_->compute(im1, im2, disp);
  // We use 64F so when converting to Eigen everything is double
  disp.convertTo(disp, CV_64F);
  disp /= std::pow(2, 4);
}
