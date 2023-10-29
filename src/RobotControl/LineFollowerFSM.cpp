#include "RobotControl/LineFollowerFSM.hpp"
#include <boost/log/keywords/channel.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include "Utilities/TimeManager.hpp"
#include <iostream>
#include <bitset>

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;

using namespace RobotCode::Utilities;

namespace RobotCode::RobotControl::LineFollowerFSM {

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

State::State() :
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("LineFollowerFSM"));
}

void State::logState(std::string stateName) {
  auto time = std::chrono::system_clock::now();
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      stateName;
}

bool State::isEnd() {
  return false;
}

bool State::isEndNormal() {
  throw std::runtime_error("This state is not an end");
}

std::string State::endStatus() {
  throw std::runtime_error("This state is not an end");
}

State &State::commonLineChooser(char sensorData) {
  if (countSetBits(sensorData) > 5) {
    return StateManager::getIntersectionState();
  } else if ((sensorData & 0x18) == 0x18) {
    return StateManager::getCenterState();
  } else if ((sensorData & 0x0F) != 0x00 && (sensorData & 0xF0) == 0x00) {
    if ((sensorData & 0x0F) > 0x03) {
      return StateManager::getTurnLeftState();
    } else {
      return StateManager::getRotateLeftState();
    }
  } else if ((sensorData & 0x0F) == 0x00 && (sensorData & 0xF0) != 0x00) {
    if ((sensorData & 0xF0) > 0x30) {
      return StateManager::getRotateRightState();
    } else {
      return StateManager::getTurnRightState();
    }
  } else {
    StateManager::getCorrectErrorState().reset();
    return StateManager::getCorrectErrorState();
  }
}

void CenterState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("Center");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                   RobotCode::RobotControl::DriveTrain::Speed::Fast);
}

State &CenterState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getCorrectErrorState();
  } else {
    return nextState;
  }
}

void TurnLeftState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("TurnLeft");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Medium);
}

State &TurnLeftState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getTurnLeftLostState();
  } else {
    return nextState;
  }
}

void RotateLeftState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("RotateLeft");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::RotateLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

State &RotateLeftState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getTurnLeftLostState();
  } else {
    return nextState;
  }
}

void TurnRightState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("TurnRight");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnRight,
                   RobotCode::RobotControl::DriveTrain::Speed::Medium);
}

State &TurnRightState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getTurnRightLostState();
  } else {
    return nextState;
  }
}

void RotateRightState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("RotateRight");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::RotateRight,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

State &RotateRightState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getTurnRightLostState();
  } else {
    return nextState;
  }
}

void TurnLeftLostState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("TurnLeftLost");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::BackTurnLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

State &TurnLeftLostState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getTurnLeftLostState();
  } else {
    return nextState;
  }
}

void TurnRightLostState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("TurnRightLost");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::BackTurnRight,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

State &TurnRightLostState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getTurnRightLostState();
  } else {
    return nextState;
  }
}

void IntersectionState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("Intersection");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

void IntersectionState::setPath(const std::vector<IntersectionDirection>& path, int pathRep) {
  m_numIntersectionsInPath = path.size();
  m_path = path;
  m_pathRep = pathRep;
  m_intersectionInPathCnt = 0;
  m_pathRepCnt = 0;
}

void IntersectionState::resetIntersectionCnt() {
  m_intersectionInPathCnt = 0;
}

State &IntersectionState::getNextState(char sensorData) {
  if (m_intersectionInPathCnt < m_numIntersectionsInPath) {
    m_intersectionInPathCnt++;
    IntersectionDirection dir = m_path[m_intersectionInPathCnt-1];
    if (dir == IntersectionDirection::Straight) {
      return StateManager::getInIntersectionForwardState();
    } else if (dir == IntersectionDirection::Left) {
      StateManager::getInIntersectionLeftState().reset();
      return StateManager::getInIntersectionLeftState();
    } else if (dir == IntersectionDirection::Right) {
      StateManager::getInIntersectionRightState().reset();
      return StateManager::getInIntersectionRightState();
    } else {
      throw std::runtime_error("Invalid direction");
    }
  } else {
    m_intersectionInPathCnt = 0;
    StateManager::getIntersectionWaitState()
        .setContinueTime(std::chrono::system_clock::now() + std::chrono::milliseconds(2000));
    return StateManager::getIntersectionWaitState();
  }
}

void InIntersectionForwardState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("InIntersection");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                   RobotCode::RobotControl::DriveTrain::Speed::Fast);
}

State &InIntersectionForwardState::getNextState(char sensorData) {
  if (countSetBits(sensorData) > 2) {
    return StateManager::getInIntersectionForwardState();
  } else {
    return commonLineChooser(sensorData);
  }
}


void InIntersectionLeftState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("InLeftIntersection");
  if (!m_reachedOffPath) {
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnLeft,
                     RobotCode::RobotControl::DriveTrain::Speed::Medium);
  } else {
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::RotateLeft,
                     RobotCode::RobotControl::DriveTrain::Speed::Medium);
  }
}

State &InIntersectionLeftState::getNextState(char sensorData) {
  if (!m_reachedOffPath && countSetBits(sensorData) > 0) {
    return StateManager::getInIntersectionLeftState();
  } else if (!m_reachedOffPath && countSetBits(sensorData) == 0) {
    m_reachedOffPath = true;
    return StateManager::getInIntersectionLeftState();
  } else if (m_reachedOffPath && countSetBits(sensorData) == 0) {
    return StateManager::getInIntersectionLeftState();
  } else if (m_reachedOffPath && (sensorData & 0x1F) == 0) {
    return StateManager::getInIntersectionLeftState();
  } else {
    return commonLineChooser(sensorData);
  }
}

void InIntersectionLeftState::reset() {
  m_reachedOffPath = false;
}

void InIntersectionRightState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("InLeftIntersection");
  if (!m_reachedOffPath) {
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnRight,
                     RobotCode::RobotControl::DriveTrain::Speed::Medium);
  } else {
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::RotateRight,
                     RobotCode::RobotControl::DriveTrain::Speed::Medium);
  }
}

State &InIntersectionRightState::getNextState(char sensorData) {
  if (!m_reachedOffPath && countSetBits(sensorData) > 0) {
    return StateManager::getInIntersectionRightState();
  } else if (!m_reachedOffPath && countSetBits(sensorData) == 0) {
    m_reachedOffPath = true;
    return StateManager::getInIntersectionRightState();
  } else if (m_reachedOffPath && countSetBits(sensorData) == 0) {
    return StateManager::getInIntersectionRightState();
  } else if (m_reachedOffPath && (sensorData & 0xF8) == 0) {
    return StateManager::getInIntersectionRightState();
  } else {
    return commonLineChooser(sensorData);
  }
}

void InIntersectionRightState::reset() {
  m_reachedOffPath = false;
}

void IntersectionWaitState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("IntersectionWait");
  driveTrain.stop();
}

State &IntersectionWaitState::getNextState(char sensorData) {
  auto timeNow = std::chrono::system_clock::now();
  if (timeNow > m_continueTime) {
    if (countSetBits(sensorData) <= 2) {
      return StateManager::getIntersectionBackupState();
    } else if (m_pathRepCnt < m_pathRep) {
      return StateManager::getPathEndState();
    } else {
      return StateManager::getEndState();
    }
  } else {
    return StateManager::getIntersectionWaitState();
  }
}

void IntersectionWaitState::setContinueTime(std::chrono::time_point<std::chrono::system_clock> continueTime) {
  m_continueTime = continueTime;
}

void StartState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("Start");
  //driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
  //                 RobotCode::RobotControl::DriveTrain::Speed::Medium);
  driveTrain.stop();
}

State &StartState::getNextState(char sensorData) {
  m_pathRepCnt++;
  m_intersectionInPathCnt++;
  std::cout << "Path rep count: " << m_pathRepCnt << std::endl;
  if (countSetBits(sensorData) > 2) {
    IntersectionDirection dir = m_path[m_intersectionInPathCnt - 1];
    if (dir == IntersectionDirection::Straight) {
      return StateManager::getInIntersectionForwardState();
    } else if (dir == IntersectionDirection::Left) {
      StateManager::getInIntersectionLeftState().reset();
      return StateManager::getInIntersectionLeftState();
    } else if (dir == IntersectionDirection::Right) {
      StateManager::getInIntersectionRightState().reset();
      return StateManager::getInIntersectionRightState();
    } else {
      throw std::runtime_error("Invalid direction");
    }
  } else {
    return commonLineChooser(sensorData);
  }
}

void CorrectErrorState::runMotors(DriveTrain driveTrain) {
  logState("CorrectError");
  if (std::chrono::system_clock::now() - m_errorStartTime <= std::chrono::milliseconds(2000) ||
      std::chrono::system_clock::now() - m_errorStartTime > std::chrono::milliseconds(5000)) {
    driveTrain.stop();
  } else if (std::chrono::system_clock::now() - m_errorStartTime <= std::chrono::milliseconds(3000))  {
    driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
  } else if (std::chrono::system_clock::now() - m_errorStartTime <= std::chrono::milliseconds(5000))  {
    driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
  }
}

State &CorrectErrorState::getNextState(char sensorData) {
  m_stateActive = true;
  if (std::chrono::system_clock::now() - m_errorStartTime <= std::chrono::milliseconds(5000)) {
    State& nextState = commonLineChooser(sensorData);
    if (&nextState != this) {
      m_stateActive = false;
    }
    return nextState;
  } else {
    m_stateActive = false;
    return StateManager::getErrorState();
  }
}

void CorrectErrorState::reset() {
  if (!m_stateActive) {
    m_errorStartTime = std::chrono::system_clock::now();
  }
}

void ErrorState::runMotors(DriveTrain driveTrain) {
  logState("Error");
  driveTrain.stop();
}

State &ErrorState::getNextState(char sensorData) {
  return commonLineChooser(sensorData);
}

bool ErrorState::isEnd() {
  return true;
}

bool ErrorState::isEndNormal() {
  return false;
}

std::string ErrorState::endStatus() {
  return "Error";
}

void IntersectionBackupState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("IntersectionBackup");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Backward,
                   RobotCode::RobotControl::DriveTrain::Speed::Slow);
}

State &IntersectionBackupState::getNextState(char sensorData) {
  if (sensorData != 0x7E) {
    return StateManager::getIntersectionBackupState();
  } else {
    StateManager::getIntersectionWaitState()
        .setContinueTime(std::chrono::system_clock::now() + std::chrono::milliseconds(2000));
    return StateManager::getIntersectionWaitState();
  }
}

void BackwardState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("Backward");
  driveTrain.drive(DriveTrain::Backward, DriveTrain::Slow);
}

State &BackwardState::getNextState(char sensorData) {
  State &nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getCorrectErrorState()) {
    return StateManager::getBackwardState();
  } else {
    return nextState;
  }
}

void EndState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("End");
  driveTrain.stop();
}

State &EndState::getNextState(char sensorData) {
  return *this;
}

bool EndState::isEnd() {
  return true;
}

bool EndState::isEndNormal() {
  return true;
}

std::string EndState::endStatus() {
  return "End";
}

void PathEndState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("PathEnd");
  driveTrain.stop();
}

State &PathEndState::getNextState(char sensorData) {
  return StateManager::getStartState();
}

CenterState &StateManager::getCenterState() {
  return centerState;
}

TurnLeftState &StateManager::getTurnLeftState() {
  return turnLeftState;
}

RotateLeftState &StateManager::getRotateLeftState() {
  return rotateLeftState;
}

TurnRightState &StateManager::getTurnRightState() {
  return turnRightState;
}

RotateRightState &StateManager::getRotateRightState() {
  return rotateRightState;
}

TurnLeftLostState &StateManager::getTurnLeftLostState() {
  return turnLeftLostState;
}

TurnRightLostState &StateManager::getTurnRightLostState() {
  return turnRightLostState;
}

IntersectionState &StateManager::getIntersectionState() {
  return intersectionState;
}

BackwardState &StateManager::getBackwardState() {
  return backwardState;
}

ErrorState &StateManager::getErrorState() {
  return errorState;
}

CorrectErrorState &StateManager::getCorrectErrorState() {
  return correctErrorState;
}

StartState &StateManager::getStartState() {
  return startState;
}

InIntersectionForwardState &StateManager::getInIntersectionForwardState() {
  return inIntersectionForwardState;
}

InIntersectionLeftState &StateManager::getInIntersectionLeftState() {
  return inIntersectionLeftState;
}

InIntersectionRightState &StateManager::getInIntersectionRightState() {
  return inIntersectionRightState;
}

IntersectionWaitState &StateManager::getIntersectionWaitState() {
  return intersectionWaitState;
}

EndState &StateManager::getEndState() {
  return endState;
}

IntersectionBackupState &StateManager::getIntersectionBackupState() {
  return intersectionBackupState;
}

PathEndState &StateManager::getPathEndState() {
  return pathEndState;
}

CenterState StateManager::centerState;
TurnLeftState StateManager::turnLeftState;
RotateLeftState StateManager::rotateLeftState;
TurnRightState StateManager::turnRightState;
RotateRightState StateManager::rotateRightState;
TurnLeftLostState StateManager::turnLeftLostState;
TurnRightLostState StateManager::turnRightLostState;
IntersectionState StateManager::intersectionState;
InIntersectionForwardState StateManager::inIntersectionForwardState;
InIntersectionLeftState StateManager::inIntersectionLeftState;
InIntersectionRightState StateManager::inIntersectionRightState;
IntersectionWaitState StateManager::intersectionWaitState;
BackwardState StateManager::backwardState;
ErrorState StateManager::errorState;
CorrectErrorState StateManager::correctErrorState;
StartState StateManager::startState;
EndState StateManager::endState;
IntersectionBackupState StateManager::intersectionBackupState;
PathEndState StateManager::pathEndState;

unsigned int State::m_intersectionInPathCnt = 0;
unsigned int State::m_numIntersectionsInPath = 0;
unsigned int State::m_pathRep = 0;
unsigned int State::m_pathRepCnt = 0;
std::vector<State::IntersectionDirection> State::m_path;

}