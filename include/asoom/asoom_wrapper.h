#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include "asoom/asoom.h"

class ASOOMWrapper {
  public:
    //! Setup variables, init stuff
    ASOOMWrapper(ros::NodeHandle& nh);

    //! Setup ROS pubs/subs
    void initialize();

  private:
    /***********************************************************
     * LOCAL FUNCTIONS
     ***********************************************************/

    //! Callback for VO.  Pass empty image pointer if only tracking pose
    void poseImgCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
        const sensor_msgs::Image::ConstPtr& img_msg);

    //! Callback for GPS data
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);

    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    ros::NodeHandle nh_;

    std::unique_ptr<ASOOM> asoom_;

    //! If true, sub to synchronized images and poses
    bool require_imgs_;

    //! ROS Pubs and subs
    std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sync_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> img_sync_sub_;
    std::unique_ptr<message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, 
      sensor_msgs::Image>> pose_img_sync_sub_;
    ros::Subscriber gps_sub_, pose_sub_;
    ros::Publisher trajectory_viz_pub_;
};
