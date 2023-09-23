#ifndef ROBOT_CODE_DEVICEMANAGERS_REFLECTANCESENSORMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_REFLECTANCESENSORMANAGER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/log/sources/channel_logger.hpp>

namespace RobotCode::DeviceManagers {


class ReflectanceSensorManager {
 public:
  explicit ReflectanceSensorManager(std::chrono::microseconds decayTime);
  ~ReflectanceSensorManager();
  void initialize();
  void startUpdateThread();
  void quickStart();
  void stopUpdateThread();
  void quickStop();
  char getSensorValues();
 private:
  char readSenors();
  void ensureInitialized() const;
  void update();
  static const int NUM_SENSORS = 8;
  constexpr static const int SENSOR_GPIO[NUM_SENSORS] =
      {25, 24, 23, 22, 27, 18, 17, 4};
  static const int LED_ON_GPIO = 5;
  std::chrono::microseconds decayTime;
  bool initialized = false;
  std::thread updateThread;
  bool threadRunning = false;
  bool threadInterrupt = false;
  std::mutex threadMutex;
  std::condition_variable threadCond;
  std::atomic_char sensorValues = 0;

  boost::log::sources::channel_logger<> m_logger;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_REFLECTANCESENSORMANAGER_HPP
