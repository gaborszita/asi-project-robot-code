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
  VideoCapture cap;

  if (!cap.open(-1)) {
    throw std::runtime_error("Failed to open video capture.");
  }

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = tagStandard41h12_create();
  apriltag_detector_add_family(td, tf);
  std::unique_lock<std::mutex> lock(threadMutex);
  while (!threadInterrupt) {
    lock.unlock();
    Mat image, grayscaleImage;
    if (!cap.read(image)) {
      throw std::runtime_error("Failed to read image from video capture.");
    }
    auto timeNow = std::chrono::system_clock::now();

    cvtColor(image, grayscaleImage, cv::COLOR_RGB2GRAY);
    image_u8_t apriltagImage = {grayscaleImage.cols, grayscaleImage.rows, grayscaleImage.cols, grayscaleImage.data};
    zarray_t *detections = apriltag_detector_detect(td, &apriltagImage);

    PositionEstimator::CameraProperties cameraProperties{};
    cameraProperties.fx = 1052.5/2;//992.8915/2;
    cameraProperties.fy = 1052.5/2;//992.8915/2;
    cameraProperties.cx = 639.5/2;
    cameraProperties.cy = 359.5/2;
    cameraProperties.x = 0.115;
    cameraProperties.y = 0;
    cameraProperties.z = 0;
    cameraProperties.roll = 180*std::numbers::pi/180;
    cameraProperties.pitch = 0;
    cameraProperties.yaw = 180*std::numbers::pi/180;

    PositionEstimator::RobotPosition robotPosition{};
    bool detected = detections->size > 0;
    if (detected) {
      std::vector<std::pair<PositionEstimator::CameraProperties, zarray_t *>> detectionsVector;
      detectionsVector.emplace_back(cameraProperties, detections);
      try {
        robotPosition = PositionEstimator::getRobotPosition(detectionsVector);
      } catch (std::runtime_error& error) {
        std::cout << "WARNING: Failed to get robot position: " << error.what() << std::endl;
        detected = false;
      }
      /*std::cout << "Robot position: " << robotPosition.x << ", " << robotPosition.y << ", " << robotPosition.z
                << std::endl;
      std::cout << "Robot rotation: " << robotPosition.roll*180/3.14 << ", " << robotPosition.pitch*180/3.14 << ", " << robotPosition.yaw*180/3.14
                << std::endl;*/
    }
    apriltag_detections_destroy(detections);

    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));



    lock.lock();
    if (detected) {
      long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
          (timeNow - TimeManager::getStartTime()).count();
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