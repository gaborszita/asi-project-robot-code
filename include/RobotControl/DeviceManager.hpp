#ifndef ROBOT_CODE_ROBOTCONTROL_DEVICEMANAGER_HPP
#define ROBOT_CODE_ROBOTCONTROL_DEVICEMANAGER_HPP

#include "DeviceManagers/MouseManager.hpp"
#include "DeviceManagers/GyroManager.hpp"
#include "logging/LidarLogManager.hpp"
#include "DeviceManagers/LidarManager.hpp"
#include "DeviceManagers/ReflectanceSensorManager.hpp"
#include "RobotControl/DriveTrain.hpp"
#include "DeviceManagers/DriveMotorManager.hpp"

namespace RobotCode::RobotControl {

class DeviceManager {
 public:
  DeviceManager();
  void startup();
  void shutdown();
  RobotCode::DeviceManagers::MouseManager &getMouseManager();
  RobotCode::DeviceManagers::GyroManager &getGyroManager();
  RobotCode::Logging::LidarLogManager &getLidarLogManager();
  RobotCode::DeviceManagers::LidarManager &getLidarManager();
  RobotCode::DeviceManagers::ReflectanceSensorManager &getReflectanceSensorManager();
  RobotCode::RobotControl::DriveTrain &getDriveTrain();
  RobotCode::DeviceManagers::DriveMotorManager &getDriveMotorManager();
 private:
  RobotCode::DeviceManagers::MouseManager mouseManager;
  RobotCode::DeviceManagers::GyroManager gyroManager;
  RobotCode::Logging::LidarLogManager lidarLogManager;
  RobotCode::DeviceManagers::ReflectanceSensorManager reflectanceSensorManager;
  RobotCode::RobotControl::DriveTrain driveTrain;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_DEVICEMANAGER_HPP