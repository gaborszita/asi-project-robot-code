#ifndef ROBOT_CODE_LOGGING_LIDARLOGMANAGER_HPP
#define ROBOT_CODE_LOGGING_LIDARLOGMANAGER_HPP

#include "DeviceManagers/LidarManager.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/log/sources/channel_logger.hpp>
#include <shared_mutex>

namespace RobotCode::Logging {

class LidarLogManager {
 public:
  LidarLogManager();
  void quickStart();
  void quickStop();
  RobotCode::DeviceManagers::LidarManager &getLidarManager();
  void getScanData(rplidar_response_measurement_node_hq_t *nodes, size_t &nodeCount);
 private:
  void update();
  RobotCode::DeviceManagers::LidarManager lidarManager;
  std::thread logThread;
  std::mutex threadMutex;
  bool threadRunning = false;
  bool threadInterrupt = false;
  boost::log::sources::channel_logger<> m_logger;

  rplidar_response_measurement_node_hq_t m_nodes[8192];
  size_t m_nodeCount = sizeof(m_nodes)/sizeof(rplidar_response_measurement_node_hq_t);
  std::shared_mutex lidarDataReadMutex;
};

}

#endif //ROBOT_CODE_LOGGING_LIDARLOGMANAGER_HPP