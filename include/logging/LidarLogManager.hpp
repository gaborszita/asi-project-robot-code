#ifndef ROBOT_CODE_LOGGING_LIDARLOGMANAGER_HPP
#define ROBOT_CODE_LOGGING_LIDARLOGMANAGER_HPP

#include "DeviceManagers/LidarManager.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/log/sources/channel_logger.hpp>

namespace RobotCode::Logging {

class LidarLogManager {
 public:
  LidarLogManager();
  void quickStart();
  void quickStop();
  RobotCode::DeviceManagers::LidarManager &getLidarManager();
 private:
  void update();
  RobotCode::DeviceManagers::LidarManager lidarManager;
  std::thread logThread;
  std::mutex threadMutex;
  bool threadRunning = false;
  bool threadInterrupt = false;
  boost::log::sources::channel_logger<> m_logger;
};

}

#endif //ROBOT_CODE_LOGGING_LIDARLOGMANAGER_HPP