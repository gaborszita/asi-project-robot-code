#ifndef ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace RobotCode::DeviceManagers {

class MouseManager {
 public:
  void quickStart();
  void startMouseThread();
  [[nodiscard]] float getX() const;
  [[nodiscard]] float getY() const;
 private:
  [[noreturn]] void update();
  bool threadRunning = false;

  std::atomic<float> x = 0;
  std::atomic<float> y = 0;
  static const std::string mouseDevicePath;
  // dots per centimeter(dpcm) NOT dots per inch(DPI),
  // convert DPI to dpcm by multiplying DPI by 0.393
  static const int MOUSE_DPCM = 441;
  static const int MOUSE_DOTS_PER_METER = MOUSE_DPCM * 100;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_MOUSEMANAGER_HPP