#include "logging/LidarLogManager.hpp"
#include <future>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/constant.hpp>
#include <sstream>
#include <boost/log/utility/manipulators/add_value.hpp>
#include "Utilities/TimeManager.hpp"
#include <iostream>

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

using namespace RobotCode::Utilities;

namespace RobotCode::Logging {

LidarLogManager::LidarLogManager() :
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("RPLidar"));
}

void LidarLogManager::quickStart() {
  lidarManager.quickStart();
  if (threadRunning) {
    throw std::logic_error("LidarLogManager update thread already running.");
  }
  threadInterrupt = false;
  logThread = std::thread(&LidarLogManager::update, this);
  threadRunning = true;
}

void LidarLogManager::quickStop() {
  if (!threadRunning) {
    throw std::logic_error("LidarLogManager update thread not running.");
  }
  threadMutex.lock();
  threadInterrupt = true;
  threadMutex.unlock();
  logThread.join();
  threadRunning = false;
  lidarManager.quickStop();
}

void LidarLogManager::update() {
  std::unique_lock<std::mutex> lock(threadMutex);
  int failedToGetScanDataCount = 0;
  while (!threadInterrupt) {
    m_nodeCount = sizeof(m_nodes)/sizeof(rplidar_response_measurement_node_hq_t);
    lock.unlock();
    std::unique_lock<std::shared_mutex> lock2(lidarDataReadMutex);
    auto timeNow = std::chrono::system_clock::now();
    try {
      lidarManager.grabScanData(m_nodes, m_nodeCount);
      failedToGetScanDataCount = 0;
    } catch (std::runtime_error& error) {
      failedToGetScanDataCount++;
      if (failedToGetScanDataCount == 10) {
        std::cout << "WARNING: Failed to get lidar scan data 10 times in a row, restarting lidar manager." << std::endl;
        lidarManager.quickStop();
        lidarManager.quickStart();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      } else if (failedToGetScanDataCount <= 20) {
        std::cout << "WARNING: Failed to get lidar scan data: " << error.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      } else {
        throw error;
      }
    }
    lock2.unlock();
    lock.lock();
    if (failedToGetScanDataCount == 0 && !threadInterrupt) {
      std::stringstream lidarDataStream;
      for (int i = 0; i < m_nodeCount; i++) {
        float angle_in_degrees = m_nodes[i].angle_z_q14 * 90.f / (1 << 14);
        float distance_in_meters = m_nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
        lidarDataStream << distance_in_meters << ","
                        << angle_in_degrees << ","
                        << unsigned(m_nodes[i].quality) << ","
                        << unsigned(m_nodes[i].flag);
        if (i != m_nodeCount - 1) {
          lidarDataStream << ",";
        }
      }

      long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
          (timeNow - TimeManager::getStartTime()).count();
      std::string lidarData = lidarDataStream.str();
      BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                          << m_nodeCount << "," << lidarData;
    }
  }
}

void LidarLogManager::getScanData(rplidar_response_measurement_node_hq_t *nodes, size_t &nodeCount) {
  std::shared_lock<std::shared_mutex> lock(lidarDataReadMutex);
  nodeCount = m_nodeCount;
  if (nodeCount < m_nodeCount) {
    throw std::logic_error("LidarLogManager node count is less than expected.");
  } else {
    for (int i = 0; i < m_nodeCount; i++) {
        nodes[i] = m_nodes[i];
    }
  }
}

RobotCode::DeviceManagers::LidarManager &LidarLogManager::getLidarManager() {
  return lidarManager;
}

}