#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/constant.hpp>
#include "DeviceManagers/DriveMotorManager.hpp"
#include <chrono>
#include <boost/log/utility/manipulators/add_value.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;

using namespace RobotCode::Utilities;

namespace RobotCode::DeviceManagers {

DriveMotorManager::DriveMotorManager()
    : m1(hat.getDC(1)),
      m2(hat.getDC(2)),
      m3(hat.getDC(3)),
      m4(hat.getDC(4)),
      m_logger(keywords::channel = "device") {
    m_logger.add_attribute("Device", attrs::constant<std::string>("DriveMotor"));
}

void DriveMotorManager::runMotor(RobotCode::DeviceManagers::DriveMotorManager::Motor motor,
                                 RobotCode::DeviceManagers::DriveMotorManager::MotorDirection direction,
                                 char speed) {
  logData(motor, direction, speed, std::chrono::system_clock::now());
  //BOOST_LOG(m_logger) << motor << "," << direction << "," << speed;
  switch (motor) {
    case FrontLeft:
      if (direction == Forward) {
        m2.run(BACKWARD);
      } else {
        m2.run(FORWARD);
      }
      m2.setSpeed(speed);
      break;
    case FrontRight:
      if (direction == Forward) {
          m1.run(BACKWARD);
      } else {
          m1.run(FORWARD);
      }
      m1.setSpeed(speed);
      break;
    case BackLeft:
      if (direction == Forward) {
          m3.run(FORWARD);
      } else {
          m3.run(BACKWARD);
      }
      m3.setSpeed(speed);
      break;
    case BackRight:
      if (direction == Forward) {
          m4.run(FORWARD);
      } else {
          m4.run(BACKWARD);
      }
      m4.setSpeed(speed);
      break;
  }
}

void DriveMotorManager::releaseMotor(RobotCode::DeviceManagers::DriveMotorManager::Motor motor) {
  switch (motor) {
    case FrontLeft:
      m2.run(RELEASE);
      break;
    case FrontRight:
      m1.run(RELEASE);
      break;
    case BackLeft:
      m3.run(RELEASE);
      break;
    case BackRight:
      m4.run(RELEASE);
      break;
  }
}

void DriveMotorManager::logData(DriveMotorManager::Motor motor,
                                DriveMotorManager::MotorDirection motorDirection,
                                char speed,
                                std::chrono::time_point<std::chrono::system_clock> time) {
  std::string motorString;
  if (motor == FrontLeft) {
    motorString = "FrontLeft";
  } else if (motor == FrontRight) {
    motorString = "FrontRight";
  } else if (motor == BackLeft) {
    motorString = "BackLeft";
  } else if (motor == BackRight) {
    motorString = "BackRight";
  } else {
    motorString = "Unknown";
  }

  std::string directionString;
  if (motorDirection == Forward) {
    directionString = "Forward";
  } else if (motorDirection == Backward) {
    directionString = "Backward";
  } else {
    directionString = "Unknown";
  }

  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();

  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                      << motorString << "," << directionString << "," << +speed;
}

}