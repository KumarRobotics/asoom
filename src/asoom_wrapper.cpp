// ROS Wrapper for ASOOM library

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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

  std::cout << "\033[32m" << "[ROS] ======== Configuration ========" << std::endl <<
    "[ROS] require_imgs: " << require_imgs_ << std::endl <<
    "[ROS] pgo_thread_period_ms: " << asoom_params.pgo_thread_period_ms << std::endl <<
    "[ROS] keyframe_dist_thresh_m: " << asoom_params.keyframe_dist_thresh_m << std::endl <<
    "[ROS] pose_graph_between_sigmas_pos: " << pg_bs_p << std::endl <<
    "[ROS] pose_graph_between_sigmas_rot: " << pg_bs_r << std::endl <<
    "[ROS] pose_graph_gps_sigmas: " << pg_gs << std::endl <<
    "[ROS] pose_graph_gps_sigma_per_sec: " << pg_gsps << std::endl <<
    "[ROS] pose_graph_fix_scale: " << pg_fs << std::endl <<
    "[ROS] pose_graph_num_frames_init: " << pg_nfi << std::endl <<
    "[ROS] ====== End Configuration ======" << "\033[0m" << std::endl;

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

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker traj_marker, cam_marker;

  traj_marker.header.frame_id = "world";
  ros::Time time;
  time.fromNSec(asoom_->getMostRecentStamp());
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

  auto end_t = high_resolution_clock::now();
  std::cout << "\033[32m" << "[ROS] Output Visualization: " <<
      duration_cast<microseconds>(end_t - start_t).count() << "us" << "\033[0m" << std::endl
        << std::flush;

  trajectory_viz_pub_.publish(marker_array);
  publishUTMTransform(time);
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
