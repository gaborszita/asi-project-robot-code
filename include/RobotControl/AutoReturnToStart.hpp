#ifndef ROBOT_CODE_ROBOTCONTROL_AUTORETURNTOSTART_HPP
#define ROBOT_CODE_ROBOTCONTROL_AUTORETURNTOSTART_HPP

#include "logging/LidarLogManager.hpp"
#include "RobotControl/DriveTrain.hpp"
#include "DeviceManagers/GyroManager.hpp"
#include "DeviceManagers/ReflectanceSensorManager.hpp"

namespace RobotCode::RobotControl {

class AutoReturnToStart {
 public:
  AutoReturnToStart(RobotCode::Logging::LidarLogManager &lidarLogManager,
                    DriveTrain &driveTrain,
                    RobotCode::DeviceManagers::GyroManager &gyroManager,
                    RobotCode::DeviceManagers::ReflectanceSensorManager &rsm);

  void returnToStart();
  void ensureStartDistance();
  void rotate();
  void driveToFrdBwdMiddle();
  void ensureNotInRgtLftMiddle();
  bool strideToPath();
  bool goToStart();
  bool verifyStart();
  void setTargetRotation(float targetRotation);
 private:
  struct LidarReadingStatus {
    bool frontTooClose;
    bool frontInvalid;
    bool backTooClose;
    bool backInvalid;
    bool rightTooClose;
    bool rightInvalid;
    bool leftTooClose;
    bool leftInvalid;
    bool frontCloser;
    bool rightCloser;
  };
  struct LidarDistances {
    float front;
    float back;
    float right;
    float left;
  };
  LidarReadingStatus checkPoint(rplidar_response_measurement_node_hq_t* nodes, size_t nodeCount);
  LidarDistances getDistances(rplidar_response_measurement_node_hq_t* nodes, size_t nodeCount);
  RobotCode::Logging::LidarLogManager &lidarLogManager;
  RobotCode::RobotControl::DriveTrain &driveTrain;
  RobotCode::DeviceManagers::GyroManager &gyroManager;
  RobotCode::DeviceManagers::ReflectanceSensorManager &reflectanceSensorManager;
  float m_targetRotation = 0;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_AUTORETURNTOSTART_HPP
