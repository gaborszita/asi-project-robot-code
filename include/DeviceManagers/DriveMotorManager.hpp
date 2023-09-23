//
// Created by Gabor Szita on 9/9/23.
//

#ifndef ROBOT_CODE_DEVICEMANAGERS_DRIVEMOTORMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_DRIVEMOTORMANAGER_HPP

#include "Adafruit_MotorHAT.h"
#include <boost/log/sources/channel_logger.hpp>
#include <ctime>
#include <chrono>

namespace RobotCode::DeviceManagers {

class DriveMotorManager {
 public:
  DriveMotorManager();
  enum Motor {
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight
  };
  enum MotorDirection {
    Forward,
    Backward
  };
  void runMotor(Motor motor, MotorDirection direction, char speed);
  void releaseMotor(Motor motor);
 private:
  Adafruit_MotorHAT hat;
  Adafruit_DCMotor& m1;
  Adafruit_DCMotor& m2;
  Adafruit_DCMotor& m3;
  Adafruit_DCMotor& m4;

  boost::log::sources::channel_logger<> m_logger;
  void logData(Motor motor, MotorDirection motorDirection, char speed,
               std::chrono::time_point<std::chrono::system_clock> time);
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_DRIVEMOTORMANAGER_HPP
