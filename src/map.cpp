#include "asoom/map.h"

Map::Map(const Config& config) {
  map_ = grid_map::GridMap({
      "elevation", 
      "color", 
      "semantics",
      "view_angle",
      "num_points"});
  // Reset layers to the appropriate values
  map_.setFrameId("world");
  map_.setGeometry(grid_map::Length(50, 50), config.resolution, grid_map::Position(0, 0));
  clear();
}

void Map::addCloud(const Eigen::Array4Xf& cloud, const Eigen::Isometry3d& camera_pose) {
  grid_map::Position corner_pos;
  map_.getPosition(grid_map::Index(0,0), corner_pos);
  // This is the array with the indices each point falls into
  Eigen::Array2Xi inds = (((-cloud).topRows<2>().colwise() + 
      corner_pos.array().cast<float>()) / map_.getResolution()).round().cast<int>();
  Eigen::Isometry3f camera_pose_inv = camera_pose.inverse().cast<float>();

  // Get references to map layers we need, speeds up access so we only do key lookup once
  grid_map::Matrix &elevation_layer = map_["elevation"];
  grid_map::Matrix &color_layer = map_["color"];
  grid_map::Matrix &semantics_layer = map_["semantics"];
  grid_map::Matrix &view_angle_layer = map_["view_angle"];
  grid_map::Matrix &num_points_layer = map_["num_points"];

  // Iterate through points
  grid_map::Index ind;
  Eigen::Vector3f pt_camera_frame;
  double view_angle;
  for (size_t col=0; col<inds.cols(); col++) {
    ind = inds.col(col);
    if ((ind < 0).any() || (ind >= map_.getSize()).any()) {
      // Point is outside map bounds
      continue;
    }
    // Cumulative mean
    elevation_layer(ind[0], ind[1]) += (cloud(2, col) - elevation_layer(ind[0], ind[1])) / 
      (num_points_layer(ind[0], ind[1]) + 1);
    num_points_layer(ind[0], ind[1])++;

    // Use color from closest to image center
    pt_camera_frame = camera_pose_inv * cloud.col(col).head<3>();
    view_angle = std::abs(std::atan2(pt_camera_frame.head<2>().norm(), pt_camera_frame[2]));
    if (view_angle < view_angle_layer(ind[0], ind[1])) {
      view_angle_layer(ind[0], ind[1]) = view_angle;
      color_layer(ind[0], ind[1]) = cloud(3, col);
    }
  }
}

void Map::clear() {
  map_.setConstant("elevation", 0);
  map_.setConstant("color", NAN);
  map_.setConstant("semantics", NAN);
  map_.setConstant("view_angle", M_PI/2);
  map_.setConstant("num_points", 0);
}

grid_map_msgs::GridMap Map::exportROSMsg() {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map_, msg);
  return msg;
}

const grid_map::GridMap &Map::getMap() const {
  return map_;
}
