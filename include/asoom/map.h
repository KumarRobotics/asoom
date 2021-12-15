#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Dense>

/*!
 * Wraps a GridMap as the underlying data structure, manages integrated map
 */
class Map {
  public: 
    struct Config {
      double resolution; // In m/cell
    };

    Map(const Config& config);

    /*!
     * Add a new pointcloud to the map.
     *
     * @cloud Pointcloud to add, should already be in the world frame
     */
    void addCloud(const Eigen::Array4Xf& cloud);

    /*!
     * Reset all map layers
     */
    void clear();

    grid_map_msgs::GridMap exportROSMsg();

  private:
    grid_map::GridMap map_;
};
