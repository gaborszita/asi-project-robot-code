#ifndef ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP
#define ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP

#include "DeviceManagers/ReflectanceSensorManager.hpp"
#include "RobotControl/DriveTrain.hpp"
#include "LineFollowerFSM.hpp"
#include <boost/log/sources/channel_logger.hpp>

namespace RobotCode::RobotControl {

class LineFollower {
 public:
  LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager& rsm, DriveTrain& driveTrain);
  void followLine();
 private:
  RobotCode::DeviceManagers::ReflectanceSensorManager &rsm;
  DriveTrain &driveTrain;
  boost::log::sources::channel_logger<> m_logger;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP