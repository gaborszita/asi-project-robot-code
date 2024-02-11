#include "AprilTags/AprilTagManager.hpp"
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/attributes/constant.hpp>
#include "Utilities/TimeManager.hpp"
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag_pose.h>
#include "AprilTags/PositionEstimator.hpp"
#include <stdexcept>
#include <iostream>
#include <numbers>

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

using namespace RobotCode::Utilities;
using namespace cv;

namespace RobotCode::AprilTags {

AprilTagManager::AprilTagManager() :
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("AprilTag"));
}

void AprilTagManager::startUpdateThread() {
  if (threadRunning) {
    throw std::logic_error("AprilTagManager update thread already running.");
  }
  threadInterrupt = false;
  updateThread = std::thread(&AprilTagManager::update, this);
  threadRunning = true;
}

void AprilTagManager::stopUpdateThread() {
  if (!threadRunning) {
    throw std::logic_error("AprilTagManager update thread not running.");
  }
  threadMutex.lock();
  threadInterrupt = true;
  threadMutex.unlock();
  threadCond.notify_all();
  updateThread.join();
  threadRunning = false;
}

void AprilTagManager::update() {
  VideoCapture capBack;
  VideoCapture capFront;

  if (!capBack.open(2)) {
    throw std::runtime_error("Failed to open back video capture.");
  } else if (!capFront.open(0)) {
    throw std::runtime_error("Failed to open front video capture.");
  }

  std::array<std::pair<VideoCapture, PositionEstimator::CameraProperties>, 2> captures;
  PositionEstimator::CameraProperties backCameraProperties{};
  backCameraProperties.fx = 1052.5/2;//992.8915/2;
  backCameraProperties.fy = 1052.5/2;//992.8915/2;
  backCameraProperties.cx = 639.5/2;
  backCameraProperties.cy = 359.5/2;
  backCameraProperties.x = 0.115;
  backCameraProperties.y = 0;
  backCameraProperties.z = 0;
  backCameraProperties.roll = 180*std::numbers::pi/180;
  backCameraProperties.pitch = 0;
  backCameraProperties.yaw = 180*std::numbers::pi/180;
  captures[0] = std::make_pair(capBack, backCameraProperties);
  PositionEstimator::CameraProperties frontCameraProperties{};
  frontCameraProperties.fx = 840.417;//992.8915/2;
  frontCameraProperties.fy = 840.417;//992.8915/2;
  frontCameraProperties.cx = 319.5;
  frontCameraProperties.cy = 239.5;
  frontCameraProperties.x = 0.092;
  frontCameraProperties.y = 0;
  frontCameraProperties.z = 0;
  frontCameraProperties.roll = 0;
  frontCameraProperties.pitch = 0;
  frontCameraProperties.yaw = 0;
  captures[1] = std::make_pair(capFront, frontCameraProperties);

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = tagStandard41h12_create();
  apriltag_detector_add_family(td, tf);
  std::unique_lock<std::mutex> lock(threadMutex);
  while (!threadInterrupt) {
    lock.unlock();
    std::vector<std::pair<PositionEstimator::CameraProperties, zarray_t *>> detectionsVector;
    auto timeNow = std::chrono::system_clock::now();
    bool detected = false;

    std::array<Mat, captures.size()> images;

    for (int i=0; i<captures.size(); i++) {
      Mat image;
      if (!captures[i].first.read(image)) {
        throw std::runtime_error("Failed to read image from video capture.");
      }
      images[i] = image;
    }

    for (int i=0; i<captures.size(); i++) {
      Mat image = images[i];
      Mat grayscaleImage;

      cvtColor(image, grayscaleImage, cv::COLOR_RGB2GRAY);
      image_u8_t apriltagImage = {grayscaleImage.cols, grayscaleImage.rows, grayscaleImage.cols, grayscaleImage.data};
      zarray_t *detections = apriltag_detector_detect(td, &apriltagImage);

      detectionsVector.emplace_back(std::make_pair(captures[i].second, detections));
      detected = detected || detections->size > 0;
    }

    PositionEstimator::RobotPosition robotPosition{};
    if (detected) {
      try {
        robotPosition = PositionEstimator::getRobotPosition(detectionsVector);
      } catch (std::runtime_error& error) {
        std::cout << "WARNING: Failed to get robot position: " << error.what() << std::endl;
        detected = false;
      }
    }

    for (int i=0; i<captures.size(); i++) {
      apriltag_detections_destroy(detectionsVector[i].second);
    }

    lock.lock();
    if (detected) {
      long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
          (timeNow - TimeManager::getStartTime()).count();
      //std::cout << "x: " << robotPosition.x << ", y: " << robotPosition.y << ", z: " << robotPosition.z << std::endl;
      //std::cout << "yaw: " << 180/3.14*robotPosition.yaw << std::endl;
      BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp",
                                                timeLog)
                          << robotPosition.x << "," << robotPosition.y << "," << robotPosition.z << ","
                          << robotPosition.roll << "," << robotPosition.pitch << "," << robotPosition.yaw << ","
                          << robotPosition.id << "," << robotPosition.err << ","
                          << robotPosition.tagRelativeX << "," << robotPosition.tagRelativeY << "," << robotPosition.tagRelativeZ;
    }
  }
}

void AprilTagManager::quickStart() {
  startUpdateThread();
}

void AprilTagManager::quickStop() {
  stopUpdateThread();
}

}