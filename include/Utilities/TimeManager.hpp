#ifndef ROBOT_CODE_UTILITIES_TIMEMANAGER_HPP
#define ROBOT_CODE_UTILITIES_TIMEMANAGER_HPP

#include <chrono>

namespace RobotCode::Utilities {

class TimeManager {
 public:
  static void resetStartTime();
  static std::chrono::time_point<std::chrono::system_clock> getStartTime();
 private:
  static std::chrono::time_point<std::chrono::system_clock> startTime;
};

}

#endif //ROBOT_CODE_UTILITIES_TIMEMANAGER_HPP