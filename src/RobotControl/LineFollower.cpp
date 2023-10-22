#include "RobotControl/LineFollower.hpp"
#include <iostream>
#include <bitset>
#include <boost/log/attributes.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/keywords/channel.hpp>
#include <boost/log/trivial.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

using namespace RobotCode::RobotControl::LineFollowerFSM;
using namespace RobotCode::Utilities;

namespace RobotCode::RobotControl {

LineFollower::LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager &rsm, DriveTrain &driveTrain) :
    rsm(rsm),
    driveTrain(driveTrain),
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("LineFollower"));
}

void LineFollower::followLine() {
  //driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Medium);
  //std::this_thread::sleep_for(std::chrono::milliseconds(3000000));
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::vector<State::IntersectionDirection> path;
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Left);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Straight);
  //path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Straight);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Right);
  path.push_back(State::IntersectionDirection::Right);
  StateManager::getIntersectionState().setPath(path, 1);
  State *currentState = &StateManager::getStartState();
  currentState->runMotors(driveTrain);
  /*driveTrain.drive(DriveTrain::Forward, DriveTrain::Medium);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Fast);
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  driveTrain.drive(DriveTrain::Backward, DriveTrain::Medium);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Fast);
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  currentState = &StateManager::getEndState();
  currentState->runMotors(driveTrain);*/
  while (!currentState->isEnd()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    char data = rsm.getSensorValues() & 0x7E;

    //std::cout << std::bitset<8>(data) << std::endl;

    currentState = &currentState->getNextState(data);
    currentState->runMotors(driveTrain);
  }
  auto time = std::chrono::system_clock::now();
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                      << "End" << "," << currentState->isEndNormal() << "," << currentState->endStatus();
}

}