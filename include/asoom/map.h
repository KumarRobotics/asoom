#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Dense>

/*!
 * Wraps a GridMap as the underlying data structure, manages integrated map
 */
class Map {
  public: 
    struct Params {
      double resolution; // In m/cell

      Params(double r) : resolution(r) {}

      Params() : Params(0.1) {}
    };

    Map(const Params& params);

    /*!
     * Add a new pointcloud to the map.
     *
     * @param cloud Pointcloud to add, should already be in the world frame
     * @param camera_pose Camera pose from which cloud projected.
     */
    void addCloud(const Eigen::Array4Xf& cloud, const Eigen::Isometry3d& camera_pose);

    /*!
     * Reset all map layers
     */
    void clear();

    grid_map_msgs::GridMap exportROSMsg();

    //! This is really just for test purposes
    const grid_map::GridMap &getMap() const;

  private:
    grid_map::GridMap map_;
};
