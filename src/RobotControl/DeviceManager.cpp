#include "RobotControl/DeviceManager.hpp"

namespace RobotCode::RobotControl {

DeviceManager::DeviceManager() :
    reflectanceSensorManager(std::chrono::microseconds(300)) {
}

RobotCode::DeviceManagers::MouseManager &DeviceManager::getMouseManager() {
  return mouseManager;
}

RobotCode::DeviceManagers::GyroManager &DeviceManager::getGyroManager() {
  return gyroManager;
}

RobotCode::Logging::LidarLogManager &DeviceManager::getLidarLogManager() {
  return lidarLogManager;
}

RobotCode::DeviceManagers::LidarManager &DeviceManager::getLidarManager() {
  return lidarLogManager.getLidarManager();
}

RobotCode::DeviceManagers::ReflectanceSensorManager &DeviceManager::getReflectanceSensorManager() {
  return reflectanceSensorManager;
}

RobotCode::RobotControl::DriveTrain &DeviceManager::getDriveTrain() {
  return driveTrain;
}

RobotCode::DeviceManagers::DriveMotorManager &DeviceManager::getDriveMotorManager() {
  return driveTrain.getDriveMotorManager();
}

RobotCode::AprilTags::AprilTagManager &DeviceManager::getAprilTagManager() {
  return aprilTagManager;
}

void DeviceManager::startup() {
  mouseManager.quickStart();
  gyroManager.quickStart();
  lidarLogManager.quickStart();
  reflectanceSensorManager.quickStart();
  aprilTagManager.quickStart();
}

void DeviceManager::shutdown() {
  mouseManager.quickStop();
  gyroManager.quickStop();
  lidarLogManager.quickStop();
  reflectanceSensorManager.quickStop();
  driveTrain.stop();
  aprilTagManager.quickStop();
}

}