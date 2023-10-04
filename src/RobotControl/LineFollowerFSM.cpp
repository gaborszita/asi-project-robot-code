#include "RobotControl/LineFollowerFSM.hpp"

namespace RobotCode::RobotControl::LineFollowerFSM {

States State::commonLineChooser(char sensorData) {
  if (sensorData == 0xF7) {
    return Intersection;
  } else if ((sensorData & 0x18) == 0x18) {
    return Center;
  } else if ((sensorData & 0x0F) != 0x00 && (sensorData & 0xF0) == 0x00) {
    if ((sensorData & 0x0F) > 0x03) {
      return TurnLeft;
    } else {
      return RotateLeft;
    }
  } else if ((sensorData & 0x0F) == 0x00 && (sensorData & 0xF0) != 0x00) {
    if ((sensorData & 0xF0) > 0x30) {
      return RotateRight;
    } else {
      return TurnRight;
    }
  } else {
    return Error;
  }
}

void CenterState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                     RobotCode::RobotControl::DriveTrain::Speed::Fast);
}

States CenterState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return Backward;
  } else {
    return nextState;
  }
}

void TurnLeftState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Medium);
}

States TurnLeftState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return TurnLeftLost;
  } else {
    return nextState;
  }
}

void RotateLeftState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::RotateLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

States RotateLeftState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return TurnLeftLost;
  } else {
    return nextState;
  }
}

void TurnRightState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnRight,
                   RobotCode::RobotControl::DriveTrain::Speed::Medium);
}

States TurnRightState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return TurnRightLost;
  } else {
    return nextState;
  }
}

void RotateRightState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::RotateRight,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

States RotateRightState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return TurnRightLost;
  } else {
    return nextState;
  }
}

void TurnLeftLostState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::BackTurnLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

States TurnLeftLostState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return TurnLeftLost;
  } else {
    return nextState;
  }
}

void TurnRightLostState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::BackTurnRight,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

States TurnRightLostState::getNextState(char sensorData) {
  States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return TurnRightLost;
  } else {
    return nextState;
  }
}

void IntersectionState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                   RobotCode::RobotControl::DriveTrain::Speed::Fast);
}


int countSetBits(int n) {
  int count = 0;

  while (n > 0) {
    // Use bitwise AND to check the least significant bit
    // If it's 1, increment the count
    count += n & 1;

    // Right shift the number to check the next bit
    n >>= 1;
  }

  return count;
}

States IntersectionState::getNextState(char sensorData) {
  if (countSetBits(sensorData) > 2) {
    return Intersection;
  } else {
    return commonLineChooser(sensorData);
  }
}

void ErrorState::runMotors(DriveTrain driveTrain) {
  driveTrain.stop();
}

States ErrorState::getNextState(char sensorData) {
  return commonLineChooser(sensorData);
}

void BackwardState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  driveTrain.drive(DriveTrain::Backward, DriveTrain::Slow);
}

States BackwardState::getNextState(char sensorData) {
    States nextState = commonLineChooser(sensorData);
  if (nextState == Error) {
    return Backward;
  } else {
    return nextState;
  }
}

}