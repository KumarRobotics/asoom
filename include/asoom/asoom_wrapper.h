#pragma once

#include <ros/ros.h>
#include "asoom/asoom.h"

class ASOOMWrapper {
  public:
    //! Setup variables, init stuff
    ASOOMWrapper(ros::NodeHandle& nh);

    //! Setup ROS pubs/subs
    void initialize();

  private:
    /***********************************************************
     * LOCAL VARIABLES
     ***********************************************************/

    ros::NodeHandle nh_;

    std::unique_ptr<ASOOM> asoom_;
};
