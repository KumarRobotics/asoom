// ROS Wrapper for ASOOM library

#include <cv_bridge/cv_bridge.h>

#include "asoom/asoom_wrapper.h"

ASOOMWrapper::ASOOMWrapper(ros::NodeHandle& nh) {
  nh_ = nh;

  nh_.param<bool>("require_imgs", require_imgs_, true);

  // Top level parameters
  ASOOM::Params asoom_params;
  nh_.param<int>("pgo_thread_period_ms", asoom_params.pgo_thread_period_ms, 1000);
  nh_.param<float>("keyframe_dist_thresh_m", asoom_params.keyframe_dist_thresh_m, 5);

  // Parameters for PGO
  double pg_bs_p, pg_bs_r, pg_gs, pg_gsps;
  bool pg_fs;
  int pg_nfi;
  nh_.param<double>("pose_graph_between_sigmas_pos", pg_bs_p, 0.1);
  nh_.param<double>("pose_graph_between_sigmas_rot", pg_bs_r, 0.1);
  nh_.param<double>("pose_graph_gps_sigmas", pg_gs, 0.1);
  nh_.param<double>("pose_graph_gps_sigma_per_sec", pg_gsps, 0.0);
  nh_.param<bool>("pose_graph_fix_scale", pg_fs, false);
  nh_.param<int>("pose_graph_num_frames_init", pg_nfi, 5);
  PoseGraph::Params pose_graph_params(pg_bs_p, pg_bs_r, pg_gs, pg_gsps, pg_fs, pg_nfi);

  asoom_ = std::make_unique<ASOOM>(asoom_params, pose_graph_params);
}

void ASOOMWrapper::initialize() {
  if (require_imgs_) {
    pose_sync_sub_ = std::make_unique<message_filters::Subscriber<geometry_msgs::PoseStamped>>(
        nh_, "pose", 50);
    img_sync_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(
        nh_, "img", 50);
    pose_img_sync_sub_ = std::make_unique<message_filters::TimeSynchronizer
      <geometry_msgs::PoseStamped, sensor_msgs::Image>>(*pose_sync_sub_, *img_sync_sub_, 50);
    pose_img_sync_sub_->registerCallback(&ASOOMWrapper::poseImgCallback, this);
  } else {
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("pose", 10, 
        std::bind(&ASOOMWrapper::poseImgCallback, this, std::placeholders::_1, 
          sensor_msgs::Image::ConstPtr()));
  }
  gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("gps", 10, &ASOOMWrapper::gpsCallback, this);

  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz", 1);
}

void ASOOMWrapper::poseImgCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
    const sensor_msgs::Image::ConstPtr& img_msg)
{
}

void ASOOMWrapper::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
}
