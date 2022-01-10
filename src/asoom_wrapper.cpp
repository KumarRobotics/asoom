// ROS Wrapper for ASOOM library

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <visualization_msgs/MarkerArray.h>

#include "asoom/asoom_wrapper.h"
#include "asoom/utils.h"

ASOOMWrapper::ASOOMWrapper(ros::NodeHandle& nh)
  : frustum_pts_(initFrustumPts(1)), nh_(nh)
{
  utm_origin_ = Eigen::Vector2d::Zero();

  nh_.param<bool>("require_imgs", require_imgs_, true);

  // Top level parameters
  ASOOM::Params asoom_params;
  nh_.param<int>("pgo_thread_period_ms", asoom_params.pgo_thread_period_ms, 1000);
  nh_.param<int>("stereo_thread_period_ms", asoom_params.stereo_thread_period_ms, 1000);
  nh_.param<int>("map_thread_period_ms", asoom_params.map_thread_period_ms, 1000);
  nh_.param<float>("keyframe_dist_thresh_m", asoom_params.keyframe_dist_thresh_m, 1);
  nh_.param<bool>("use_semantics", asoom_params.use_semantics, false);
  nh_.param<std::string>("semantic_lut_path", asoom_params.semantic_lut_path, 
      SemanticColorLut::NO_SEM);
  use_semantics_ = asoom_params.use_semantics;

  // Parameters for PGO
  double pg_bs_p, pg_bs_r, pg_gs, pg_gsps;
  bool pg_fs;
  int pg_nfi;
  nh_.param<double>("pose_graph_between_sigmas_pos", pg_bs_p, 0.05);
  nh_.param<double>("pose_graph_between_sigmas_rot", pg_bs_r, 0.05);
  nh_.param<double>("pose_graph_gps_sigmas", pg_gs, 0.5);
  nh_.param<double>("pose_graph_gps_sigma_per_sec", pg_gsps, 0.5);
  nh_.param<bool>("pose_graph_fix_scale", pg_fs, false);
  nh_.param<int>("pose_graph_num_frames_init", pg_nfi, 5);
  PoseGraph::Params pose_graph_params(pg_bs_p, pg_bs_r, pg_gs, pg_gsps, pg_fs, pg_nfi);

  // Parameters for Rectifier
  std::string r_calib_path;
  float r_scale;
  if (require_imgs_) {
    nh_.param<std::string>("rectifier_calib_path", r_calib_path, "");
  } else {
    r_calib_path = Rectifier::NO_IMGS;
  }
  nh_.param<float>("rectifier_scale", r_scale, 0.5);
  Rectifier::Params rectifier_params(r_calib_path, r_scale);

  //Parameters for Stereo
  DenseStereo::Params stereo_params;
  nh_.param<int>("stereo_min_disparity", stereo_params.min_disparity, 1);
  nh_.param<int>("stereo_num_disparities", stereo_params.num_disparities, 80);
  nh_.param<int>("stereo_block_size", stereo_params.block_size, 9);
  nh_.param<int>("stereo_P1_coeff", stereo_params.P1_coeff, 1);
  nh_.param<int>("stereo_P2_coeff", stereo_params.P2_coeff, 3);
  nh_.param<int>("stereo_disp_12_map_diff", stereo_params.disp_12_map_diff, 0);
  nh_.param<int>("stereo_pre_filter_cap", stereo_params.pre_filter_cap, 35);
  nh_.param<int>("stereo_uniqueness_ratio", stereo_params.uniqueness_ratio, 10);
  nh_.param<int>("stereo_speckle_window_size", stereo_params.speckle_window_size, 100);
  nh_.param<int>("stereo_speckle_range", stereo_params.speckle_range, 20);

  // Parameters for Map
  Map::Params map_params;
  nh_.param<double>("map_resolution", map_params.resolution, 0.5);
  nh_.param<double>("map_buffer_size_m", map_params.buffer_size_m, 50);

  if (asoom_params.use_semantics && 
      asoom_params.semantic_lut_path == SemanticColorLut::NO_SEM) 
  {
    std::cout << "\033[31m" << 
      "[WARNING] Using semantics, but no semantic color LUT is provided." << 
      "\033[0m" << std::endl;
  }

  std::cout << "\033[32m" << "[ROS] ======== Configuration ========" << std::endl <<
    "[ROS] require_imgs: " << require_imgs_ << std::endl <<
    "[ROS] pgo_thread_period_ms: " << asoom_params.pgo_thread_period_ms << std::endl <<
    "[ROS] stereo_thread_period_ms: " << asoom_params.stereo_thread_period_ms << std::endl <<
    "[ROS] map_thread_period_ms: " << asoom_params.map_thread_period_ms << std::endl <<
    "[ROS] keyframe_dist_thresh_m: " << asoom_params.keyframe_dist_thresh_m << std::endl <<
    "[ROS] use_semantics: " << asoom_params.use_semantics << std::endl <<
    "[ROS] semantic_lut_path: " << asoom_params.semantic_lut_path << std::endl <<
    "[ROS] ===============================" << std::endl <<
    "[ROS] pose_graph_between_sigmas_pos: " << pg_bs_p << std::endl <<
    "[ROS] pose_graph_between_sigmas_rot: " << pg_bs_r << std::endl <<
    "[ROS] pose_graph_gps_sigmas: " << pg_gs << std::endl <<
    "[ROS] pose_graph_gps_sigma_per_sec: " << pg_gsps << std::endl <<
    "[ROS] pose_graph_fix_scale: " << pg_fs << std::endl <<
    "[ROS] pose_graph_num_frames_init: " << pg_nfi << std::endl <<
    "[ROS] ===============================" << std::endl <<
    "[ROS] rectifier_calib_path: " << r_calib_path << std::endl <<
    "[ROS] rectifier_scale: " << r_scale << std::endl <<
    "[ROS] ===============================" << std::endl <<
    "[ROS] stereo_min_disparity: " << stereo_params.min_disparity << std::endl <<
    "[ROS] stereo_num_disparities:" << stereo_params.num_disparities << std::endl <<
    "[ROS] stereo_block_size: " << stereo_params.block_size << std::endl <<
    "[ROS] stereo_P1_coeff: " << stereo_params.P1_coeff << std::endl << 
    "[ROS] stereo_P2_coeff: " << stereo_params.P2_coeff << std::endl << 
    "[ROS] stereo_disp_12_map_diff: " << stereo_params.disp_12_map_diff << std::endl << 
    "[ROS] stereo_pre_filter_cap: " << stereo_params.pre_filter_cap << std::endl << 
    "[ROS] stereo_uniqueness_ratio:" << stereo_params.uniqueness_ratio << std::endl << 
    "[ROS] stereo_speckle_window_size: " << stereo_params.speckle_window_size << std::endl << 
    "[ROS] stereo_speckle_range: " << stereo_params.speckle_range << std::endl << 
    "[ROS] ===============================" << std::endl <<
    "[ROS] map_resolution: " << map_params.resolution << std::endl << 
    "[ROS] map_buffer_size_m: " << map_params.buffer_size_m << std::endl << 
    "[ROS] ====== End Configuration ======" << "\033[0m" << std::endl;

  asoom_ = std::make_unique<ASOOM>(asoom_params, pose_graph_params, rectifier_params, 
      stereo_params, map_params);
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

  if (use_semantics_) {
    sem_sub_ = nh_.subscribe<sensor_msgs::Image>("sem", 10, &ASOOMWrapper::semCallback, this);
  }

  gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("gps", 10, &ASOOMWrapper::gpsCallback, this);

  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz", 1);
  recent_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("recent_cloud", 1);
  map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map", 1);

  output_timer_ = nh_.createTimer(ros::Duration(1.0), &ASOOMWrapper::outputCallback, this);
}

std::vector<Eigen::Vector3d> ASOOMWrapper::initFrustumPts(float scale) {
  std::vector<Eigen::Vector3d> pts;
  pts.push_back(scale*Eigen::Vector3d(0, 0, 0));
  pts.push_back(scale*Eigen::Vector3d(1, 0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(0, 0, 0));
  pts.push_back(scale*Eigen::Vector3d(-1, 0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(0, 0, 0));
  pts.push_back(scale*Eigen::Vector3d(-1, -0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(0, 0, 0));
  pts.push_back(scale*Eigen::Vector3d(1, -0.625, 1));

  pts.push_back(scale*Eigen::Vector3d(1, 0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(-1, 0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(-1, 0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(-1, -0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(-1, -0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(1, -0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(1, -0.625, 1));
  pts.push_back(scale*Eigen::Vector3d(1, 0.625, 1));
  return pts;
}

void ASOOMWrapper::outputCallback(const ros::TimerEvent& event) {
  using namespace std::chrono;

  auto start_t = high_resolution_clock::now();
  ros::Time time;
  time.fromNSec(asoom_->getMostRecentStamp());
  publishPoseGraphViz(time);
  publishRecentPointCloud(time);
  publishUTMTransform(time);
  map_pub_.publish(asoom_->getMapMessage());
  auto end_t = high_resolution_clock::now();

  std::cout << "\033[32m" << "[ROS] Output Visualization: " <<
      duration_cast<microseconds>(end_t - start_t).count() << "us" << "\033[0m" << std::endl
        << std::flush;
}

void ASOOMWrapper::publishPoseGraphViz(const ros::Time& time) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker traj_marker, cam_marker;

  traj_marker.header.frame_id = "world";
  traj_marker.header.stamp = time;
  traj_marker.ns = "trajectory";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
  traj_marker.action = visualization_msgs::Marker::ADD;
  traj_marker.pose.position.x = 0;
  traj_marker.pose.position.y = 0;
  traj_marker.pose.position.z = 0;
  traj_marker.pose.orientation.x = 0;
  traj_marker.pose.orientation.y = 0;
  traj_marker.pose.orientation.z = 0;
  traj_marker.pose.orientation.w = 1;
  traj_marker.scale.x = 0.2; // Line width
  traj_marker.color.a = 1;
  traj_marker.color.r = 1;
  traj_marker.color.g = 0;
  traj_marker.color.b = 0;

  cam_marker.header = traj_marker.header;
  cam_marker.ns = "cams";
  cam_marker.id = 0;
  cam_marker.type = visualization_msgs::Marker::LINE_LIST;
  cam_marker.action = visualization_msgs::Marker::ADD;
  cam_marker.pose = traj_marker.pose;
  cam_marker.scale.x = 0.1; // Line width
  cam_marker.color.a = 1;
  cam_marker.color.r = 0;
  cam_marker.color.g = 1;
  cam_marker.color.b = 0;

  std::vector<Eigen::Isometry3d> poses = asoom_->getGraph();
  std::cout << "\033[32m" << "[ROS] Keyframe count: " << poses.size() << "\033[0m" << std::endl;

  for (const auto& pose : poses) {
    traj_marker.points.push_back(Eigen2ROS(pose.translation()));

    for (const auto& pt : frustum_pts_) {
      cam_marker.points.push_back(Eigen2ROS(pose * pt));
    }
  }

  marker_array.markers.push_back(traj_marker);
  marker_array.markers.push_back(cam_marker);

  trajectory_viz_pub_.publish(marker_array);
}

void ASOOMWrapper::publishRecentPointCloud(const ros::Time& time) {
  long stamp = asoom_->getMostRecentStampWithDepth();
  if (stamp < 0) {
    // No keyframes with depth yet
    return;
  }
  DepthCloudArray pc = asoom_->getDepthCloud(stamp);

  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = time;
  cloud.header.frame_id = "world";
  cloud.height = 1;
  cloud.width = pc.cols();

  sensor_msgs::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::PointField::FLOAT32;
  x_field.count = 1;
  cloud.fields.push_back(x_field);

  sensor_msgs::PointField y_field;
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::PointField::FLOAT32;
  y_field.count = 1;
  cloud.fields.push_back(y_field);

  sensor_msgs::PointField z_field;
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::PointField::FLOAT32;
  z_field.count = 1;
  cloud.fields.push_back(z_field);

  sensor_msgs::PointField rgb_field;
  rgb_field.name = "rgb";
  rgb_field.offset = 12;
  rgb_field.datatype = sensor_msgs::PointField::FLOAT32;
  rgb_field.count = 1;
  cloud.fields.push_back(rgb_field);

  sensor_msgs::PointField class_field;
  class_field.name = "class";
  class_field.offset = 16;
  class_field.datatype = sensor_msgs::PointField::FLOAT32;
  class_field.count = 1;
  cloud.fields.push_back(class_field);

  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = true;

  cloud.data.insert(cloud.data.end(), reinterpret_cast<char *>(pc.data()), 
      reinterpret_cast<char *>(pc.data()) + cloud.row_step*cloud.height);

  recent_cloud_pub_.publish(cloud);
}

void ASOOMWrapper::publishUTMTransform(const ros::Time& time) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped trans;

  trans.header.stamp = time;
  trans.header.frame_id = "world";
  trans.child_frame_id = "utm";
  trans.transform.rotation.x = 0;
  trans.transform.rotation.y = 0;
  trans.transform.rotation.z = 0;
  trans.transform.rotation.w = 1;

  trans.transform.translation.x = -utm_origin_[0];
  trans.transform.translation.y = -utm_origin_[1];
  trans.transform.translation.z = 0;

  br.sendTransform(trans);
}

void ASOOMWrapper::poseImgCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
    const sensor_msgs::Image::ConstPtr& img_msg)
{
  Eigen::Isometry3d pose = ROS2Eigen(*pose_msg);
  cv::Mat img;
  if (img_msg) {
    img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  asoom_->addFrame(pose_msg->header.stamp.toNSec(), img, pose);
}

void ASOOMWrapper::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
  Eigen::Vector3d gps = ROS2Eigen(*gps_msg);
  if (utm_origin_[0] == 0) {
    utm_origin_ = gps.head<2>();
  }
  // PGO works better with smaller numbers instead of in utm frame directly
  gps.head<2>() -= utm_origin_;
  asoom_->addGPS(gps_msg->header.stamp.toNSec(), gps);
}

void ASOOMWrapper::semCallback(const sensor_msgs::Image::ConstPtr& sem_msg) {
  cv::Mat img;
  if (sem_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
      sem_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
    // Color
    img = cv_bridge::toCvCopy(sem_msg, sensor_msgs::image_encodings::BGR8)->image;
  } else if (sem_msg->encoding == sensor_msgs::image_encodings::MONO8 ||
             sem_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    // Gray (values are class indices)
    img = cv_bridge::toCvCopy(sem_msg, sensor_msgs::image_encodings::MONO8)->image;
  }
  asoom_->addSemantics(sem_msg->header.stamp.toNSec(), img);
}

geometry_msgs::Point ASOOMWrapper::Eigen2ROS(const Eigen::Vector3d& pos) {
  geometry_msgs::Point point;
  point.x = pos[0];
  point.y = pos[1];
  point.z = pos[2];
  return point;
}

Eigen::Isometry3d ASOOMWrapper::ROS2Eigen(const geometry_msgs::PoseStamped& pose_msg) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z
        ));
  pose.rotate(Eigen::Quaterniond(
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z
        ));
  return pose;
}

Eigen::Vector3d ASOOMWrapper::ROS2Eigen(const sensor_msgs::NavSatFix& gps_msg) {
  Eigen::Vector2d gps_latlong(gps_msg.latitude, gps_msg.longitude);  
  Eigen::Vector3d gps;
  int zone;
  gps.head<2>() = LatLong2UTM(gps_latlong, zone);
  gps[2] = gps_msg.altitude;
  return gps;
}
