#ifndef ROBOT_CODE_APRILTAGS_APRILTAGMANAGER_HPP
#define ROBOT_CODE_APRILTAGS_APRILTAGMANAGER_HPP

#include <boost/log/sources/channel_logger.hpp>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace RobotCode::AprilTags {

class AprilTagManager {
 public:
  AprilTagManager();
  void quickStart();
  void startUpdateThread();
  void quickStop();
  void stopUpdateThread();
  [[nodiscard]] double getX() const;
  [[nodiscard]] double getY() const;
 private:
  void update();

  std::thread updateThread;
  bool threadRunning = false;
  bool threadInterrupt = false;
  std::mutex threadMutex;
  std::condition_variable threadCond;

  boost::log::sources::channel_logger<> m_logger;
};

}

#endif //ROBOT_CODE_APRILTAGS_APRILTAGMANAGER_HPP