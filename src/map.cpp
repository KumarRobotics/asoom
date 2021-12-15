#include "asoom/map.h"

Map::Map(const Config& config) {
  map_ = grid_map::GridMap({
      "elevation", 
      "color", 
      "semantics",
      "view_angle",
      "num_points"});
  // Reset layers to the appropriate values
  clear();
  map_.setFrameId("world");
  map_.setGeometry(grid_map::Length(50, 50), config.resolution, grid_map::Position(0, 0));
}

void Map::addCloud(const Eigen::Array4Xf& cloud) {
}

void Map::clear() {
  map_.setConstant("elevation", NAN);
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
