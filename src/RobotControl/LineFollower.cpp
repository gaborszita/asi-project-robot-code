#include "RobotControl/LineFollower.hpp"
#include <iostream>
#include <bitset>
#include <boost/log/attributes.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/keywords/channel.hpp>
#include <boost/log/trivial.hpp>
#include "Utilities/TimeManager.hpp"
#include "RobotControl/AutoReturnToStart.hpp"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

using namespace RobotCode::RobotControl::LineFollowerFSM;
using namespace RobotCode::Utilities;

namespace RobotCode::RobotControl {

LineFollower::LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager &rsm,
                           DriveTrain &driveTrain,
                           RobotCode::DeviceManagers::GyroManager& gyroManager,
                           Logging::LidarLogManager& lidarLogManager) :
    rsm(rsm),
    driveTrain(driveTrain),
    gyroManager(gyroManager),
    lidarLogManager(lidarLogManager),
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("LineFollower"));
}

void LineFollower::followLine() {
  //driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Medium);
  //std::this_thread::sleep_for(std::chrono::milliseconds(3000000));
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::vector<State::IntersectionDirection> path;
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Left);
  StateManager::getIntersectionState().setPath(path, 200);
  AutoReturnToStart as(lidarLogManager, driveTrain, gyroManager, rsm);
  float lastStartAngle = gyroManager.getGyroZ();
  as.setTargetRotation(lastStartAngle);
  if (!as.verifyStart()) {
    as.returnToStart();
  }
  std::chrono::time_point<std::chrono::system_clock> lastIntersectionTime = std::chrono::system_clock::now();
  State *currentState = &StateManager::getStartState();
  currentState->runMotors(driveTrain);
  while (!currentState->isEnd()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    char data = rsm.getSensorValues() & 0x7E;

    currentState = &currentState->getNextState(data);

    bool verifyResult = false;
    if (currentState == &StateManager::getPathEndState() || currentState == &StateManager::getEndState()) {
      verifyResult = as.verifyStart();
    }

    if (verifyResult) {
      lastStartAngle = gyroManager.getGyroZ();
      as.setTargetRotation(lastStartAngle);
    }

    if ((currentState->isEnd() && !currentState->isEndNormal()) ||
        (currentState == &StateManager::getPathEndState()) && !verifyResult) {
      as.returnToStart();
      StateManager::getIntersectionState().resetIntersectionCnt();
      StateManager::getIntersectionState().incrementPathRep();
      currentState = &StateManager::getStartState();
    }

    if (currentState == &StateManager::getEndState() && !verifyResult) {
      currentState = &StateManager::getErrorState();
    }

    auto timeNow = std::chrono::system_clock::now();

    if (currentState == &StateManager::getIntersectionState() ||
        currentState == &StateManager::getStartState()) {
      lastIntersectionTime = timeNow;
    }

    if (timeNow - lastIntersectionTime > std::chrono::seconds (20) &&
        currentState != &StateManager::getPathEndState() &&
        currentState != &StateManager::getEndState() &&
        currentState != &StateManager::getErrorState()) {
      std::cout << "Robot stuck detected, auto return to start" << std::endl;
      BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeNow) <<
                          "Robot stuck detected";
      as.returnToStart();
      StateManager::getIntersectionState().resetIntersectionCnt();
      StateManager::getIntersectionState().incrementPathRep();
      currentState = &StateManager::getStartState();
    }

    currentState->runMotors(driveTrain);
  }
  auto time = std::chrono::system_clock::now();
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                      << "End" << "," << currentState->isEndNormal() << "," << currentState->endStatus();
}

}