#include "Utilities/TimeManager.hpp"

namespace RobotCode::Utilities {

std::chrono::time_point<std::chrono::system_clock> TimeManager::startTime;

void TimeManager::resetStartTime() {
  startTime = std::chrono::system_clock::now();
}

std::chrono::time_point<std::chrono::system_clock> TimeManager::getStartTime() {
  return startTime;
}

}