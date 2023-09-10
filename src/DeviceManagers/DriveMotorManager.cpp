#include "DeviceManagers/DriveMotorManager.hpp"

namespace RobotCode::DeviceManagers {

DriveMotorManager::DriveMotorManager()
    : m1(hat.getDC(1)),
      m2(hat.getDC(2)),
      m3(hat.getDC(3)),
      m4(hat.getDC(4)) {
}

void DriveMotorManager::runMotor(RobotCode::DeviceManagers::DriveMotorManager::Motor motor,
                                 RobotCode::DeviceManagers::DriveMotorManager::MotorDirection direction,
                                 char speed) {
  Direction forwardDir = FORWARD;
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

}