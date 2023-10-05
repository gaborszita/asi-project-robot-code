#include "RobotControl/LineFollower.hpp"
#include <iostream>
#include <bitset>
#include "RobotControl/LineFollowerFSM.hpp"
#include <boost/log/attributes.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/keywords/channel.hpp>
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
  StateManager::getIntersectionState().setPath(6, 3);
  State* currentState = &StateManager::getStartState();
  while (!currentState->isEnd()) {
    char data = rsm.getSensorValues() & 0x7E;

    //std::cout << std::bitset<8>(data) << std::endl;

    currentState = &currentState->getNextState(data);
    currentState->runMotors(driveTrain);

    if (!currentState->isEnd()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  auto time = std::chrono::system_clock::now();
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                      << "End" << currentState->isEndNormal() << "," << currentState->endStatus();
}

}