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

State& State::commonLineChooser(char sensorData) {
  if (sensorData == 0x7E) {
    std::cout << "intersectionstate" << std::endl;
    return StateManager::getIntersectionState();
  } else if ((sensorData & 0x18) == 0x18) {
    //std::cout << "centerstate" << std::endl;
    return StateManager::getCenterState();
  } else if ((sensorData & 0x0F) != 0x00 && (sensorData & 0xF0) == 0x00) {
    //std::cout << "turnleftstate" << std::endl;
    if ((sensorData & 0x0F) > 0x03) {
      return StateManager::getTurnLeftState();
    } else {
      return StateManager::getRotateLeftState();
    }
  } else if ((sensorData & 0x0F) == 0x00 && (sensorData & 0xF0) != 0x00) {
    //std::cout << "turnrightstate" << std::endl;
    if ((sensorData & 0xF0) > 0x30) {
      return StateManager::getRotateRightState();
    } else {
      return StateManager::getTurnRightState();
    }
  } else {
    //std::cout << "errorstate" << std::endl;
    return StateManager::getErrorState();
  }
}

void CenterState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("Center");
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                     RobotCode::RobotControl::DriveTrain::Speed::Fast);
}

State& CenterState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
    return StateManager::getBackwardState();
  } else {
    return nextState;
  }
}

void TurnLeftState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
  logState("TurnLeft");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::TurnLeft,
                   RobotCode::RobotControl::DriveTrain::Speed::Medium);
}

State& TurnLeftState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
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

State& RotateLeftState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
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

State& TurnRightState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
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

State& RotateRightState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
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

State& TurnLeftLostState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
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

State& TurnRightLostState::getNextState(char sensorData) {
  State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
    return StateManager::getTurnRightLostState();
  } else {
    return nextState;
  }
}

void IntersectionState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
    logState("Intersection");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                   RobotCode::RobotControl::DriveTrain::Speed::Fast);
}

void IntersectionState::setPath(int numIntersectionsInPath, int pathRep) {
  m_numIntersectionsInPath = numIntersectionsInPath;
  m_pathRep = pathRep;
  m_intersectionInPathCnt = 0;
  m_pathRepCnt = 0;
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

State& IntersectionState::getNextState(char sensorData) {
  std::cout << m_intersectionInPathCnt << std::endl;
  if (m_intersectionInPathCnt < m_numIntersectionsInPath) {
    m_intersectionInPathCnt++;
    return StateManager::getInIntersectionState();
  } else {
    m_intersectionInPathCnt = 0;
    StateManager::getIntersectionWaitState()
      .setContinueTime(std::chrono::system_clock::now() + std::chrono::milliseconds(2000));
    return StateManager::getIntersectionWaitState();
  }
}

void InIntersectionState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
    logState("InIntersection");
  driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                   RobotCode::RobotControl::DriveTrain::Speed::Fast);
}

State& InIntersectionState::getNextState(char sensorData) {
  if (countSetBits(sensorData) > 2) {
    return StateManager::getInIntersectionState();
  } else {
    return commonLineChooser(sensorData);
  }
}

void IntersectionWaitState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
    logState("IntersectionWait");
  driveTrain.stop();
}

State& IntersectionWaitState::getNextState(char sensorData) {
  auto timeNow = std::chrono::system_clock::now();
  if (timeNow > m_continueTime) {
    if (m_pathRepCnt < m_pathRep) {
      return StateManager::getStartState();
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
    driveTrain.drive(RobotCode::RobotControl::DriveTrain::Direction::Forward,
                     RobotCode::RobotControl::DriveTrain::Speed::Medium);
}

State& StartState::getNextState(char sensorData) {
  std::cout << "start" << std::endl;
  std::cout << m_intersectionInPathCnt << std::endl;
  m_pathRepCnt++;
  m_intersectionInPathCnt++;
  if (countSetBits(sensorData) > 2) {
    return StateManager::getInIntersectionState();
  } else {
    return commonLineChooser(sensorData);
  }
}

void ErrorState::runMotors(DriveTrain driveTrain) {
  logState("Error");
  driveTrain.stop();
}

State& ErrorState::getNextState(char sensorData) {
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

State& IntersectionBackupState::getNextState(char sensorData) {
  if (sensorData != 0x7E) {
    return StateManager::getIntersectionBackupState();
  } else {
    return StateManager::getIntersectionWaitState();
  }
}

void BackwardState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
    logState("Backward");
  driveTrain.drive(DriveTrain::Backward, DriveTrain::Slow);
}

State& BackwardState::getNextState(char sensorData) {
    State& nextState = commonLineChooser(sensorData);
  if (&nextState == &StateManager::getErrorState()) {
    return StateManager::getBackwardState();
  } else {
    return nextState;
  }
}

void EndState::runMotors(RobotCode::RobotControl::DriveTrain driveTrain) {
    logState("End");
  driveTrain.stop();
}

State& EndState::getNextState(char sensorData) {
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

CenterState& StateManager::getCenterState() {
  return centerState;
}

TurnLeftState& StateManager::getTurnLeftState() {
  return turnLeftState;
}

RotateLeftState& StateManager::getRotateLeftState() {
  return rotateLeftState;
}

TurnRightState& StateManager::getTurnRightState() {
  return turnRightState;
}

RotateRightState& StateManager::getRotateRightState() {
  return rotateRightState;
}

TurnLeftLostState& StateManager::getTurnLeftLostState() {
  return turnLeftLostState;
}

TurnRightLostState& StateManager::getTurnRightLostState() {
  return turnRightLostState;
}

IntersectionState& StateManager::getIntersectionState() {
  return intersectionState;
}

BackwardState& StateManager::getBackwardState() {
  return backwardState;
}

ErrorState& StateManager::getErrorState() {
  return errorState;
}

StartState& StateManager::getStartState() {
  return startState;
}

InIntersectionState &StateManager::getInIntersectionState() {
    return inIntersectionState;
}

IntersectionWaitState &StateManager::getIntersectionWaitState() {
  return intersectionWaitState;
}

EndState& StateManager::getEndState() {
  return endState;
}

IntersectionBackupState& StateManager::getIntersectionBackupState() {
  return intersectionBackupState;
}

CenterState StateManager::centerState;
TurnLeftState StateManager::turnLeftState;
RotateLeftState StateManager::rotateLeftState;
TurnRightState StateManager::turnRightState;
RotateRightState StateManager::rotateRightState;
TurnLeftLostState StateManager::turnLeftLostState;
TurnRightLostState StateManager::turnRightLostState;
IntersectionState StateManager::intersectionState;
InIntersectionState StateManager::inIntersectionState;
IntersectionWaitState StateManager::intersectionWaitState;
BackwardState StateManager::backwardState;
ErrorState StateManager::errorState;
StartState StateManager::startState;
EndState StateManager::endState;
IntersectionBackupState StateManager::intersectionBackupState;

int State::m_intersectionInPathCnt = 0;
int State::m_numIntersectionsInPath = 0;
int State::m_pathRep = 0;
int State::m_pathRepCnt = 0;

}