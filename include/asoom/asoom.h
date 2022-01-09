#pragma once

#include <thread>
#include <shared_mutex>

#include "asoom/pose_graph.h"
#include "asoom/rectifier.h"
#include "asoom/dense_stereo.h"
#include "asoom/keyframe.h"
#include "asoom/map.h"

/*!
 * Manager class for system
 *
 * Keep track of pose graph, frames, and map
 */
class ASOOM {
  public:
    struct Params {
      //! Periods for threads to run at in ms
      int pgo_thread_period_ms;
      int stereo_thread_period_ms;
      int map_thread_period_ms;

      //! Distance threshold for creating new keyframe in meters
      float keyframe_dist_thresh_m;

      //! If true, require semantic segmentation for images before stereo
      bool use_semantics;

      Params(int ptpm = 1000, int stpm = 1000, int mtps = 1000, float kdtm = 5, 
          bool us = false) :
        pgo_thread_period_ms(ptpm), stereo_thread_period_ms(stpm), 
        map_thread_period_ms(mtps), keyframe_dist_thresh_m(kdtm), use_semantics(us) {}
    };

    /*!
     * @param asoom_params High level params
     * @param pg_params Params forwarded to PoseGraph
     * @param rect_params Params forwarded to Rectifier
     * @param stereo_params Params forwarded to DenseStereo
     * @param map_params Params forwarded to Map
     */
    ASOOM(const Params& asoom_params, const PoseGraph::Params& pg_params,
      const Rectifier::Params& rect_params, const DenseStereo::Params& stereo_params,
      const Map::Params& map_params);
    ~ASOOM();

    /*!
     * Add new image frame to mapper
     *
     * @param stamp Timestamp in nsec since epoch
     * @param img Image associated with frame
     * @param pose Pose of the image in VO/VIO frame
     */
    void addFrame(long stamp, const cv::Mat& img, const Eigen::Isometry3d& pose);

    /*!
     * Add GPS measurement to mapper
     *
     * @param stamp Timestamp in nsec since epoch
     * @param pos GPS position in utm coord
     */
    void addGPS(long stamp, const Eigen::Vector3d& pos);

    /*!
     * Add semantic image to mapper
     *
     * @param stamp Timestamp in nsec since epoch.  Must exactly match frame
     * @param sem Semantically segmented image
     */
    void addSemantics(long stamp, const cv::Mat& sem);

    /*!
     * Get the current graph, might not yet be entirely optimized
     *
     * @return Vector of keyframe poses, sorted last to most recent
     */
    std::vector<Eigen::Isometry3d> getGraph(); 

    //! Get the depth cloud for the given stamp
    DepthCloudArray getDepthCloud(long stamp);

    /*!
     * Get the most recent timestamp in the graph
     *
     * @return Most recent timestamp in nsec from epoch
     */
    long getMostRecentStamp() const;

    /*! 
     * Similar to getMostRecentStamp, but require extant depth
     *
     * @return Timestamp in nsec.  Returns -1 if no keyframes with depth
     */
    long getMostRecentStampWithDepth();

    //! Get the last grid map
    grid_map_msgs::GridMap getMapMessage();

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    //! Most recent timestamp for data added to graph
    long most_recent_stamp_ = 0;

    //! Parameters for high level system
    const Params params_;

    //! Input buffer for keyframes
    struct KeyframeInput {
      std::mutex m;
      // This is a ptr since it may be moved into keyframes_
      std::list<std::unique_ptr<Keyframe>> buf;
    } keyframe_input_;

    struct GPS {
      long stamp;
      Eigen::Vector3d utm;

      GPS(long s, const Eigen::Vector3d& u) : stamp(s), utm(u) {}
    };
    //! Input buffer for GPS data
    struct GPSInput {
      std::mutex m;
      std::list<GPS> buf;
    } gps_input_;

    struct SemanticImage {
      long stamp;
      cv::Mat sem;

      SemanticImage(long s, const cv::Mat& se) : stamp(s), sem(se) {}
    };
    //! Input buffer for semantic segmentation images
    struct SemanticInput {
      std::mutex m;
      std::list<SemanticImage> buf;
    } semantic_input_;

    //! Vector of keyframes.  Important to keep indices synchronized with PoseGraph
    struct KeyframesStruct {
      std::shared_mutex m;
      Keyframes frames;
    } keyframes_;

    //! ROS Message of the current map
    struct MapMessage {
      std::mutex m;
      grid_map_msgs::GridMap msg;
    } grid_map_msg_;

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

    //! Thread managing image rectification and depth computation
    std::thread stereo_thread_;
    class StereoThread {
      public:
        StereoThread(ASOOM *a, const Rectifier::Params& rp, const DenseStereo::Params& dsp) try : 
          asoom_(a), rectifier_(rp), dense_stereo_(dsp) {
        } catch (const std::exception& ex) {
          // This usually happens when there is a yaml reading error
          std::cout << "\033[31m" << "[ERROR] Cannot create StereoThread: " << ex.what() 
            << "\033[0m" << std::endl;
        }

        bool operator()();
      private:
        void parseSemanticBuffer();

        std::vector<Keyframe> getKeyframesToCompute();

        void computeDepths(std::vector<Keyframe>& frames);

        void updateKeyframes(const std::vector<Keyframe>& frames);

        Rectifier rectifier_;

        DenseStereo dense_stereo_;

        //! Pointer back to parent
        ASOOM * const asoom_;

        bool use_semantics_;
    };

    //! Thread managing map building
    std::thread map_thread_;
    class MapThread {
      public:
        MapThread(ASOOM *a, const Map::Params& p) : asoom_(a), map_(p) {}

        bool operator()();
      private:
        std::vector<Keyframe> getKeyframesToCompute();

        void resizeMap(std::vector<Keyframe>& frames);

        void updateMap(std::vector<Keyframe>& frames);

        Map map_;

        //! Pointer back to parent
        ASOOM * const asoom_;
    };
};
