#pragma once

#include "asoom/pose_graph.h"
#include "asoom/keyframe.h"

/*!
 * Manager class for system
 *
 * Keep track of pose graph, frames, and map
 */
class ASOOM {
  public:
    //! @param pg_params Params forwarded to PoseGraph
    ASOOM(const PoseGraph::Params& pg_params);

    /*!
     * Add new image frame to mapper
     *
     * @param stamp Timestamp in nsec since epoch
     * @param img Image associated with frame
     * @param pose Pose of the image in VO/VIO frame
     */
    void addFrame(long stamp, cv::Mat& img, const Eigen::Isometry3d& pose);

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    // pg_ and keyframes_ are shared since will be co-managed by mapper
    //! PoseGraph object we are managing
    std::shared_ptr<PoseGraph> pg_;

    //! Vector of keyframes.  Important to keep indices synchronized with PoseGraph
    std::shared_ptr<Keyframes> keyframes_;
};
