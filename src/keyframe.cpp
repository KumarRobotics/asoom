#include <iostream>
#include <fstream>
#include "asoom/keyframe.h"
#include "asoom/semantic_color_lut.h"

DepthCloudArray Keyframe::getDepthCloud() const {
  if (!hasDepth()) {
    return DepthCloudArray::Zero(5, 0);
  }

  // transform to global frame
  Eigen::Isometry3d trans = getRectPose();

  DepthCloudArray cloud(5, depth_->cols());
  size_t dense_ind;
  size_t sparse_ind = 0;
  cv::Vec3b bgr;
  for (size_t x=0; x<rect_img_.size().width; x++) {
    for (size_t y=0; y<rect_img_.size().height; y++) {
      // Get row major index
      dense_ind = y*rect_img_.size().width + x;

      // Undefined depth
      if ((*depth_)(2, dense_ind) > 80 || (*depth_)(2, dense_ind) < 1) {
        continue;
      }
      
      // The weird PCL color format
      bgr = rect_img_.at<cv::Vec3b>(y, x);
      uint32_t color_packed = SemanticColorLut::packColor(bgr[0], bgr[1], bgr[2]);
      cloud.block<3,1>(0, sparse_ind) = (trans * depth_->col(dense_ind)).cast<float>();
      cloud(3, sparse_ind) = *reinterpret_cast<float*>(&color_packed);

      // We pack in the class integer in the same way
      uint8_t sem_class = 255;
      if (hasSem() && sem_img_.size() == rect_img_.size()) {
        sem_class = sem_img_.at<uint8_t>(y, x);
      }
      cloud(4, sparse_ind) = sem_class; 

      sparse_ind++;
    }
  }

  cloud.conservativeResize(Eigen::NoChange_t(), sparse_ind);
  return cloud;
}

bool Keyframe::needsMapUpdate() const {
  Eigen::Isometry3d diff = pose_.inverse() * map_pose_;
  if (diff.translation().norm() > 1 || 
      Eigen::AngleAxisd(diff.rotation()).angle() > 5*M_PI/180) {
    return true;
  }
  return false;
}

void Keyframe::saveImageBinary(const cv::Mat& img) {
}

void Keyframe::saveToDisk() {
  // This is pretty ugly.  We are going to literally stream the data to disk.
  // This is arguably rather unsafe, but it's very fast, because we don't have
  // to deal with formatting the data when we are reading/writing.
  
  std::ostringstream name; 
  name << "frame_" << stamp_ << ".bin";
  std::ofstream outfile(name.str(), std::ofstream::binary);

  {
    // Write header
    int rows = img_.size().height;
    outfile.write(reinterpret_cast<char *>(&rows), sizeof(rows));
    int cols = img_.size().width;
    outfile.write(reinterpret_cast<char *>(&cols), sizeof(cols));
    int type = img_.type();
    outfile.write(reinterpret_cast<char *>(&type), sizeof(type));

    size_t size = img_.total() * img_.elemSize();
    outfile.write(reinterpret_cast<char *>(img_.data), size);
  }
}

void Keyframe::loadFromDisk() {
}
