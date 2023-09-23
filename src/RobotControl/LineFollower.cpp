#include "RobotControl/LineFollower.hpp"

namespace RobotCode::RobotControl {

LineFollower::LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager &rsm, DriveTrain &driveTrain) :
    rsm(rsm),
    driveTrain(driveTrain) {

}

void LineFollower::followLine() {

}

}