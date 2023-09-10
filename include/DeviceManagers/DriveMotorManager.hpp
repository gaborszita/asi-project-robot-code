//
// Created by Gabor Szita on 9/9/23.
//

#ifndef ROBOT_CODE_DEVICEMANAGERS_DRIVEMOTORMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_DRIVEMOTORMANAGER_HPP

#include "Adafruit_MotorHAT.h"

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
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_DRIVEMOTORMANAGER_HPP
