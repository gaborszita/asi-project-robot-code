#ifndef ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP
#define ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP

#include "DeviceManagers/ReflectanceSensorManager.hpp"
#include "RobotControl/DriveTrain.hpp"

namespace RobotCode::RobotControl {

class LineFollower {
 public:
  LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager& rsm, DriveTrain& driveTrain);
  void followLine();
 private:
  RobotCode::DeviceManagers::ReflectanceSensorManager& rsm;
  DriveTrain& driveTrain;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP