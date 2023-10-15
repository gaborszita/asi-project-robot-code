#include "RobotControl/DriveTrain.hpp"
#include <chrono>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <boost/log/attributes/constant.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

using namespace RobotCode::Utilities;

namespace RobotCode::RobotControl {

DriveTrain::DriveTrain() :
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("DriveTrain"));
  stop();
}

void DriveTrain::drive(RobotCode::RobotControl::DriveTrain::Direction direction,
                       RobotCode::RobotControl::DriveTrain::Speed speed) {
  char realSpeed;
  if (speed == Slow) {
    realSpeed = 110;
  } else if (speed == Medium) {
    realSpeed = 155;
  } else {
    realSpeed = 200;
  }
  logData(direction, realSpeed, std::chrono::system_clock::now());
  switch (direction) {
    case Forward:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed-10);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed-10);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed-10);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed-10);
      break;
    case Backward:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      break;
    case TurnLeft:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            0);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            0);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      break;
    case TurnRight:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            0);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            0);
      break;
    case RotateLeft:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      break;
    case RotateRight:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      break;
    case BackTurnLeft:
        motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                                DeviceManagers::DriveMotorManager::Backward,
                                realSpeed);
        motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                                DeviceManagers::DriveMotorManager::Backward,
                                0);
        motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                                DeviceManagers::DriveMotorManager::Backward,
                                realSpeed);
        motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                                DeviceManagers::DriveMotorManager::Backward,
                                0);
        break;
    case BackTurnRight:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            0);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            0);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      break;
    case StrideLeft:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      break;
    case StrideRight:
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontLeft,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::FrontRight,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackLeft,
                            DeviceManagers::DriveMotorManager::Backward,
                            realSpeed);
      motorManager.runMotor(DeviceManagers::DriveMotorManager::BackRight,
                            DeviceManagers::DriveMotorManager::Forward,
                            realSpeed);
      break;
  }
}

void DriveTrain::stop() {
  logDataStop(std::chrono::system_clock::now());
  motorManager.releaseMotor(DeviceManagers::DriveMotorManager::FrontLeft);
  motorManager.releaseMotor(DeviceManagers::DriveMotorManager::FrontRight);
  motorManager.releaseMotor(DeviceManagers::DriveMotorManager::BackLeft);
  motorManager.releaseMotor(DeviceManagers::DriveMotorManager::BackRight);
}

void DriveTrain::logData(RobotCode::RobotControl::DriveTrain::Direction direction,
                         char speed,
                         std::chrono::time_point<std::chrono::system_clock> time) {
  std::string directionString;
  if (direction == Forward) {
    directionString = "Forward";
  } else if (direction == Backward) {
    directionString = "Backward";
  } else if (direction == TurnLeft) {
    directionString = "TurnLeft";
  } else if (direction == TurnRight) {
    directionString = "TurnRight";
  } else if (direction == RotateLeft) {
    directionString = "RotateLeft";
  } else if (direction == RotateRight) {
    directionString = "RotateRight";
  } else {
    directionString = "Unknown";
  }

  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                      << directionString << "," << +speed;
}

void DriveTrain::logDataStop(std::chrono::time_point<std::chrono::system_clock> time) {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                      << "Stop";
}

DeviceManagers::DriveMotorManager &DriveTrain::getDriveMotorManager() {
  return motorManager;
}

}