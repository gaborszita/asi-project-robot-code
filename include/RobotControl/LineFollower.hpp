#ifndef ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP
#define ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP

#include "DeviceManagers/ReflectanceSensorManager.hpp"
#include "RobotControl/DriveTrain.hpp"
#include "DeviceManagers/GyroManager.hpp"
#include "logging/LidarLogManager.hpp"
#include "LineFollowerFSM.hpp"
#include <boost/log/sources/channel_logger.hpp>

namespace RobotCode::RobotControl {

class LineFollower {
 public:
  LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager &rsm,
               DriveTrain &driveTrain,
               RobotCode::DeviceManagers::GyroManager& gyroManager,
               Logging::LidarLogManager& lidarLogManager);
  void followLine(const std::vector<LineFollowerFSM::State::IntersectionDirection>& path, int pathRep);
 private:
  RobotCode::DeviceManagers::ReflectanceSensorManager &rsm;
  DriveTrain &driveTrain;
  RobotCode::DeviceManagers::GyroManager& gyroManager;
  Logging::LidarLogManager& lidarLogManager;
  boost::log::sources::channel_logger<> m_logger;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWER_HPP