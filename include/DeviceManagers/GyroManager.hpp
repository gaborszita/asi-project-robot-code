#ifndef ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP

namespace RobotCode::DeviceManagers {

class GyroManager {
 public:
  void initialize();
  float getXAngle();
 private:
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP
