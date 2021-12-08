// ROS Wrapper for ASOOM library

#include "asoom/asoom_wrapper.h"

ASOOMWrapper::ASOOMWrapper(ros::NodeHandle& nh) {
  nh_ = nh;
  asoom_ = std::make_unique<ASOOM>(ASOOM::Params(), PoseGraph::Params(0.1, 0.1, 0.1));
}

void ASOOMWrapper::initialize() {
}
