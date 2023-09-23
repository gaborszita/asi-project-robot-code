#include "logging/LidarLogManager.hpp"
#include <future>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/constant.hpp>
#include <sstream>
#include <boost/log/utility/manipulators/add_value.hpp>
#include "Utilities/TimeManager.hpp"

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
  while (true) {
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
    lock.unlock();
    lidarManager.grabScanData(nodes, nodeCount);
    lock.lock();
    if (threadInterrupt) {
      break;
    }
    auto timeNow = std::chrono::system_clock::now();

    std::stringstream lidarDataStream;
    for (int i = 0; i < nodeCount; i++) {
      float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
      float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
      lidarDataStream << distance_in_meters << ","
                      << angle_in_degrees << ","
                      << unsigned(nodes[i].quality) << ","
                      << unsigned(nodes[i].flag);
      if (i != nodeCount - 1) {
        lidarDataStream << ",";
      }
    }

    long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
        (timeNow - TimeManager::getStartTime()).count();
    std::string lidarData = lidarDataStream.str();
    BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                        << nodeCount << "," << lidarData;
  }
}

RobotCode::DeviceManagers::LidarManager &LidarLogManager::getLidarManager() {
  return lidarManager;
}

}