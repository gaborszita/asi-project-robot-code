#ifndef ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <boost/log/sources/channel_logger.hpp>

namespace RobotCode::DeviceManagers {

class MouseManager {
 public:
  MouseManager();
  void quickStart();
  void startMouseThread();
  void quickStop();
  void stopMouseThread();
  [[nodiscard]] float getX() const;
  [[nodiscard]] float getY() const;
 private:
  void update();
  bool threadRunning = false;
  bool threadStopQueued = false;

  std::atomic<float> x = 0;
  std::atomic<float> y = 0;
  static const std::string mouseDevicePath;
  // dots per centimeter(dpcm) NOT dots per inch(DPI),
  // convert DPI to dpcm by multiplying DPI by 0.393
  static const int MOUSE_DPCM = 441;
  static const int MOUSE_DOTS_PER_METER = MOUSE_DPCM * 100;

  boost::log::sources::channel_logger<> m_logger;
  bool threadInterrupt = false;
  std::mutex threadMutex;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP