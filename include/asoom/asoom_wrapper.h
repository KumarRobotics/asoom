#pragma once

#include <ros/ros.h>

class ASOOMWrapper {
  public:
    ASOOMWrapper(ros::NodeHandle& nh);
    void initialize();

  private:
    ros::NodeHandle nh_;
};
