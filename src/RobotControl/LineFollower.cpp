#include "RobotControl/LineFollower.hpp"
#include <iostream>
#include <bitset>
#include "RobotControl/LineFollowerFSM.hpp"

using namespace RobotCode::RobotControl::LineFollowerFSM;

namespace RobotCode::RobotControl {

LineFollower::LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager &rsm, DriveTrain &driveTrain) :
    rsm(rsm),
    driveTrain(driveTrain) {

}

void LineFollower::followLine() {
  currentState = Center;
  while(true) {
    char data = rsm.getSensorValues() & 0x7F;

    switch (currentState) {
      case Center:
        currentState = centerState.getNextState(data);
        break;
        case TurnLeft:
        currentState = turnLeftState.getNextState(data);
        break;
        case RotateLeft:
        currentState = rotateLeftState.getNextState(data);
        break;
        case TurnRight:
        currentState = turnRightState.getNextState(data);
        break;
        case RotateRight:
        currentState = rotateRightState.getNextState(data);
        break;
        case TurnLeftLost:
        currentState = turnLeftLostState.getNextState(data);
        break;
        case TurnRightLost:
        currentState = turnRightLostState.getNextState(data);
        break;
        case Intersection:
        currentState = intersectionState.getNextState(data);
        break;
      case Backward:
        currentState = centerState.getNextState(data);
        break;
      case Error:
        currentState = errorState.getNextState(data);
        break;
    }

    if (currentState == Error) {
      std::cout << "Error" << std::endl;
      //break;
    }

    // run motors based on current state
    switch (currentState) {
      case Center:
        std::cout << "Center" << std::endl;
        centerState.runMotors(driveTrain);
        break;
        case TurnLeft:
          std::cout << "TurnLeft" << std::endl;
        turnLeftState.runMotors(driveTrain);
        break;
        case RotateLeft:
          std::cout << "RotateLeft" << std::endl;
        rotateLeftState.runMotors(driveTrain);
        break;
        case TurnRight:
          std::cout << "TurnRight" << std::endl;
        turnRightState.runMotors(driveTrain);
        break;
        case RotateRight:
          std::cout <<  "RotateRight" << std::endl;
        rotateRightState.runMotors(driveTrain);
        break;
        case TurnLeftLost:
          std::cout << "TurnLeftLost" << std::endl;
        turnLeftLostState.runMotors(driveTrain);
        break;
        case TurnRightLost:
          std::cout << "TurnRightLost" << std::endl;
        turnRightLostState.runMotors(driveTrain);
        break;
        case Intersection:
          std::cout << "Intersection" << std::endl;
        intersectionState.runMotors(driveTrain);
        break;
      case Backward:
        std::cout << "Backward" << std::endl;
        backwardState.runMotors(driveTrain);
        break;
      case Error:
        std::cout << "Error" << std::endl;
        errorState.runMotors(driveTrain);
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}