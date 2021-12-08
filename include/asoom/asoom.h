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
    struct Params {
      //! Period for PGO thread to run in ms
      int pgo_thread_period_ms;

      //! Distance threshold for creating new keyframe in meters
      float keyframe_dist_thresh_m;

      Params(int ptpm = 1000, float kdtm = 5)
        : pgo_thread_period_ms(ptpm), keyframe_dist_thresh_m(kdtm) {}
    };

    /*!
     * @param asoom_params High level params
     * @param pg_params Params forwarded to PoseGraph
     */
    ASOOM(const Params& asoom_params, const PoseGraph::Params& pg_params);
    ~ASOOM();

    /*!
     * Add new image frame to mapper
     *
     * @param stamp Timestamp in nsec since epoch
     * @param img Image associated with frame
     * @param pose Pose of the image in VO/VIO frame
     */
    void addFrame(long stamp, cv::Mat& img, const Eigen::Isometry3d& pose);

    /*!
     * Add GPS measurement to mapper
     *
     * @param stamp Timestamp in nsec since epoch
     * @param pos GPS position in utm coord
     */
    void addGPS(long stamp, const Eigen::Vector3d& pos);

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

    //! Parameters for high level system
    const Params params_;

    //! Input buffer for keyframes
    struct KeyframeInput {
      std::mutex m;
      // This is a ptr since it may be moved into keyframes_
      std::list<std::shared_ptr<Keyframe>> buf;
    } keyframe_input_;

    struct GPS {
      long stamp;
      Eigen::Vector3d utm;

      GPS(long s, Eigen::Vector3d u) : stamp(s), utm(u) {}
    };
    //! Input buffer for GPS data
    struct GPSInput {
      std::mutex m;
      std::list<GPS> buf;
    } gps_input_;

    //! Vector of keyframes.  Important to keep indices synchronized with PoseGraph
    struct KeyframesStruct {
      std::mutex m;
      Keyframes frames;
    } keyframes_;

    //! Set to true to kill all running threads
    std::atomic<bool> exit_threads_flag_ = false;

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
        ASOOM * const asoom_;

        //! Last pose, used for determining whether to create keyframe
        Eigen::Vector3d last_key_pos_;
    };
};
