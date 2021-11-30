#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

using gtsam::symbol_shorthand::P;

/*!
 * Wrapper for GTSAM Pose Graph
 */
class PoseGraph {
  public:
    PoseGraph();

    /*!
     * Create new image frame in the pose graph
     *
     * @param stamp Timestamp in nsec
     * @param pose Pose of image as computed by VO up to scale
     * @return ID number of the new frame in the graph
     */
    size_t addFrame(long stamp, const Eigen::Isometry3d& pose);

    /*!
     * Add GPS factor to graph.  Automatically linearly interpolates
     *
     * @param stamp Timestamp in nsec
     * @param gps_pose GPS pose in UTM coordinates
     */
    void addGPS(long stamp, const Eigen::Vector3d& utm_pose);

    //! Run gtsam optimization
    void update();

    /*!
     * Get pose of particular node in graph
     *
     * @param stamp Timestamp in nsec to get pose of
     * @return Pose of node.  If no node at timestamp, return nothing
     */
    std::optional<Eigen::Isometry3d> getPoseAtTime(long stamp) const;

    /*!
     * Get pose of particular node in graph
     *
     * @param ind Index of node to get pose of
     * @return Pose of node.  If no node at index, return nothing
     */
    Eigen::Isometry3d getPoseAtIndex(size_t ind) const;

    /*!
     * @return The overall scale of the graph
     */
    double getScale() const;

    /*!
     * @return Number of nodes in graph
     */
    size_t size() const;
  private:
    /*!
     * Convert GTSAM pose to Eigen
     */
    inline static gtsam::Pose3 Eigen2GTSAM(const Eigen::Isometry3d& eigen_pose) {
      return gtsam::Pose3(eigen_pose.matrix());
    }

    /*!
     * Convert Eigen pose to GTSAM
     */
    inline static Eigen::Isometry3d GTSAM2Eigen(const gtsam::Pose3& gtsam_pose) {
      return Eigen::Isometry3d(gtsam_pose.matrix());
    }

    //! Pointer to gtsam factor graph
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph_;

    //! Keep track of the current best estimates for nodes
    gtsam::Values current_opt_;

    typedef struct OriginalPose {
      OriginalPose(const Eigen::Isometry3d& p, const gtsam::Key &k) : pose(p), key(k) {}
      Eigen::Isometry3d pose;
      gtsam::Key key;
    } OriginalPose;

    //! Map to keep track of timestamp to original poses
    std::map<long, std::shared_ptr<OriginalPose>> pose_history_;

    //! Number of frames in graph
    size_t size_;
    
    //! Index of initial origin factor before GPS
    int initial_pose_factor_id_;
};

