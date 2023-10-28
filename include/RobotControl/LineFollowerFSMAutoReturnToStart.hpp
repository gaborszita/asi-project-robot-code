#ifndef ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWERFSMAUTORETURNTOSTART_HPP
#define ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWERFSMAUTORETURNTOSTART_HPP

#include "RobotControl/LineFollowerFSM.hpp"

namespace RobotCode::RobotControl::LineFollowerFSMAutoReturnToStart {

class State {
 public:
  State();
  virtual void runMotors(DriveTrain driveTrain) = 0;
  virtual State &getNextState(char sensorData) = 0;
  virtual bool isEnd();
  virtual bool isEndNormal();
  virtual std::string endStatus();

  enum IntersectionDirection {
    Left,
    Right,
    Straight,
    TurnAround
  };

 protected:
  static State &commonLineChooser(char sensorData);
  boost::log::sources::channel_logger<> m_logger;
  void logState(std::string stateName);

  static unsigned int m_numIntersectionsInPath;
  static unsigned int m_pathRep;
  static unsigned int m_intersectionInPathCnt;
  static unsigned int m_pathRepCnt;
  static std::vector<IntersectionDirection> m_path;
};

class StartState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class CenterState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class TurnLeftState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class RotateLeftState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class TurnRightState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class RotateRightState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class TurnLeftLostState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class TurnRightLostState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class IntersectionState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;

  void setPath(const std::vector<IntersectionDirection>& path, int pathRep);
};

class InIntersectionForwardState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class InIntersectionLeftState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  void reset();

 private:
  bool m_reachedOffPath = false;
};

class InIntersectionRightState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  void reset();

 private:
  bool m_reachedOffPath = false;
};

class InIntersectionTurnAroundState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  void reset();

 private:
  bool m_reachedOffPath = false;
};

class IntersectionWaitState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  void setContinueTime(std::chrono::time_point<std::chrono::system_clock> continueTime);
 private:
  std::chrono::time_point<std::chrono::system_clock> m_continueTime;
};

class CorrectErrorState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  void reset();

 private:
  std::chrono::time_point<std::chrono::system_clock> m_errorStartTime;
  bool m_stateActive = false;
};

class ErrorState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  bool isEnd() override;
  bool isEndNormal() override;
  std::string endStatus() override;
};

class BackwardState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class EndState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
  bool isEnd() override;
  bool isEndNormal() override;
  std::string endStatus() override;
};

class IntersectionBackupState : public State {
 public:
  void runMotors(DriveTrain driveTrain) override;
  State &getNextState(char sensorData) override;
};

class StateManager {
 public:
  static CenterState &getCenterState();
  static TurnLeftState &getTurnLeftState();
  static RotateLeftState &getRotateLeftState();
  static TurnRightState &getTurnRightState();
  static RotateRightState &getRotateRightState();
  static TurnLeftLostState &getTurnLeftLostState();
  static TurnRightLostState &getTurnRightLostState();
  static IntersectionState &getIntersectionState();
  static InIntersectionForwardState &getInIntersectionForwardState();
  static InIntersectionLeftState &getInIntersectionLeftState();
  static InIntersectionRightState &getInIntersectionRightState();
  static IntersectionWaitState &getIntersectionWaitState();
  static BackwardState &getBackwardState();
  static ErrorState &getErrorState();
  static CorrectErrorState &getCorrectErrorState();
  static StartState &getStartState();
  static EndState &getEndState();
  static IntersectionBackupState &getIntersectionBackupState();

 private:
  static CenterState centerState;
  static TurnLeftState turnLeftState;
  static RotateLeftState rotateLeftState;
  static TurnRightState turnRightState;
  static RotateRightState rotateRightState;
  static TurnLeftLostState turnLeftLostState;
  static TurnRightLostState turnRightLostState;
  static IntersectionState intersectionState;
  static InIntersectionForwardState inIntersectionForwardState;
  static InIntersectionLeftState inIntersectionLeftState;
  static InIntersectionRightState inIntersectionRightState;
  static BackwardState backwardState;
  static ErrorState errorState;
  static CorrectErrorState correctErrorState;
  static StartState startState;
  static EndState endState;
  static IntersectionWaitState intersectionWaitState;
  static IntersectionBackupState intersectionBackupState;
};

}

#endif //ROBOT_CODE_ROBOTCONTROL_LINEFOLLOWERFSMAUTORETURNTOSTART_HPP
