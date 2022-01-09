#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Dense>
#include "asoom/keyframe.h"

/*!
 * Wraps a GridMap as the underlying data structure, manages integrated map
 */
class Map {
  public: 
    struct Params {
      double resolution; // In m/cell
      double buffer_size_m;

      Params(double r, double bsm) : resolution(r), buffer_size_m(bsm) {}

      Params() : Params(0.1, 50) {}
    };

    Map(const Params& params);

    /*!
     * Add a new pointcloud to the map.
     *
     * @param cloud Pointcloud to add, should already be in the world frame
     * @param camera_pose Camera pose from which cloud projected.
     * @param stamp Timestamp in nsec of cloud
     */
    void addCloud(const DepthCloudArray& cloud, const Eigen::Isometry3d& camera_pose, 
        long stamp);

    /*!
     * Reset all map layers
     */
    void clear();

    /*!
     * Resize so min and max are in bounds up to buffer
     */
    void resizeToBounds(const Eigen::Vector2d& min, const Eigen::Vector2d& max);

    grid_map_msgs::GridMap exportROSMsg();

    //! This is really just for test purposes
    const grid_map::GridMap &getMap() const;

  private:
    const Params params_;

    grid_map::GridMap map_;

    //! Keep track of most recent timestamp for image in map
    long most_recent_stamp_;
};
