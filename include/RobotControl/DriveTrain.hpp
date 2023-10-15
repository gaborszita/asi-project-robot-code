#ifndef ROBOT_CODE_ROBOTCONTROL_DRIVETRAIN_HPP
#define ROBOT_CODE_ROBOTCONTROL_DRIVETRAIN_HPP

#include "DeviceManagers/DriveMotorManager.hpp"
#include <boost/log/sources/channel_logger.hpp>
#include <ctime>

namespace RobotCode::RobotControl {

class DriveTrain {
 public:
  enum Direction {
    Forward,
    Backward,
    TurnLeft,
    TurnRight,
    RotateLeft,
    RotateRight,
    BackTurnLeft,
    BackTurnRight,
    StrideLeft,
    StrideRight
  };
  enum Speed {
    Slow,
    Medium,
    Fast
  };
  DriveTrain();
  void drive(Direction direction, Speed speed);
  void stop();
  RobotCode::DeviceManagers::DriveMotorManager &getDriveMotorManager();
 private:
  RobotCode::DeviceManagers::DriveMotorManager motorManager;
  boost::log::sources::channel_logger<> m_logger;
  void logData(Direction direction, char speed, std::chrono::time_point<std::chrono::system_clock> time);
  void logDataStop(std::chrono::time_point<std::chrono::system_clock> time);
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_DRIVETRAIN_HPP