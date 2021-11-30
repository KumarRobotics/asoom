#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "top_down_render");
  ros::NodeHandle nh("~");
  return 0;
}
