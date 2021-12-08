#pragma once

#include <thread>

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

    /*!
     * Get the current graph, might not yet be entirely optimized
     *
     * @return Vector of keyframe poses, sorted last to most recent
     */
    std::vector<Eigen::Isometry3d> getGraph(); 

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    //! Input buffer for keyframes
    struct KeyframeInput {
      std::mutex m;
      std::list<std::shared_ptr<Keyframe>> buf;
    } keyframe_input_;

    //! Vector of keyframes.  Important to keep indices synchronized with PoseGraph
    struct KeyframesStruct {
      std::mutex m;
      Keyframes frames;
    } keyframes_;

    /***********************************************************
     * THREAD FUNCTORS
     ***********************************************************/

    //! Thread managing Pose Graph optimization
    std::thread pose_graph_thread_;
    class PoseGraphThread {
      public:
        // We have to pass a pointer to parent back to access local members
        // If the pointer is deleted then the class holding the thread is also
        // dead, so this seems safe, if rather ugly
        PoseGraphThread(ASOOM *a, const PoseGraph::Params& p) : asoom_(a), pg_(p) {}

        bool operator()();
      private:
        //! Work through the keyframe input buffer
        void parseBuffer();

        //! Update keyframes_ object with latest poses from PGO
        void updateKeyframes();

        //! PoseGraph object we are managing
        PoseGraph pg_;

        //! Pointer back to parent
        ASOOM *asoom_;

        //! Last pose, used for determining whether to create keyframe
        Eigen::Vector3d last_key_pos_;
    };
};
