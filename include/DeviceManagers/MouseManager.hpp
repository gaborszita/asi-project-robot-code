#ifndef ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>

namespace RobotCode::DeviceManagers {

class MouseManager {
 public:
  ~MouseManager();
  void quickStart();
  void quickStop();
  void startMouseThread();
  void stopMouseThread();
  [[nodiscard]] float getX() const;
  [[nodiscard]] float getY() const;
 private:
  void update();
  std::thread updateThread;
  bool threadRunning = false;
  bool threadInterrupt = false;
  std::mutex threadMutex;
  std::condition_variable threadCond;

  float x = 0;
  float y = 0;
  static const std::string mouseDevicePath;
  // dots per centimeter(dpcm) NOT dots per inch(DPI),
  // convert DPI to dpcm by multiplying DPI by 0.393
  static const int MOUSE_DPCM = 441;
  static const int MOUSE_DOTS_PER_METER = MOUSE_DPCM * 100;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP