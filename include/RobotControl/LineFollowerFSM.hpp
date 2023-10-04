#ifndef ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWERFSM_HPP
#define ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWERFSM_HPP

#include "DriveTrain.hpp"

namespace RobotCode::RobotControl::LineFollowerFSM {

enum States {
  Center,
  TurnLeft,
  RotateLeft,
  TurnRight,
  RotateRight,
  TurnLeftLost,
  TurnRightLost,
  Intersection,
  Backward,
  Error
};

class State {
 public:
  virtual void runMotors(DriveTrain driveTrain) = 0;
  virtual States getNextState(char sensorData) = 0;

 protected:
  static States commonLineChooser(char sensorData);
};

class CenterState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class TurnLeftState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class RotateLeftState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class TurnRightState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class RotateRightState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class TurnLeftLostState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class TurnRightLostState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class IntersectionState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class ErrorState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  States getNextState(char sensorData) override;
};

class BackwardState : public State {
    public:
    void runMotors(DriveTrain driveTrain) override;
    States getNextState(char sensorData) override;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWERFSM_HPP