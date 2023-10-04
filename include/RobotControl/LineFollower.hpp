#ifndef ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP
#define ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP

#include "DeviceManagers/ReflectanceSensorManager.hpp"
#include "RobotControl/DriveTrain.hpp"
#include "LineFollowerFSM.hpp"

namespace RobotCode::RobotControl {

class LineFollower {
 public:
  LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager& rsm, DriveTrain& driveTrain);
  void followLine();
 private:
  RobotCode::DeviceManagers::ReflectanceSensorManager &rsm;
  DriveTrain &driveTrain;

  LineFollowerFSM::CenterState centerState;
  LineFollowerFSM::TurnLeftState turnLeftState;
  LineFollowerFSM::RotateLeftState rotateLeftState;
  LineFollowerFSM::TurnRightState turnRightState;
  LineFollowerFSM::RotateRightState rotateRightState;
  LineFollowerFSM::TurnLeftLostState turnLeftLostState;
  LineFollowerFSM::TurnRightLostState turnRightLostState;
  LineFollowerFSM::IntersectionState intersectionState;
  LineFollowerFSM::BackwardState backwardState;
  LineFollowerFSM::ErrorState errorState;
  LineFollowerFSM::States currentState;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP