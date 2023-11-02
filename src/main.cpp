#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <iostream>
#include <fstream>

#include "rplidar.h"
#include "Adafruit_MotorHAT.h"
#include "DeviceManagers/GyroManager.hpp"
#include "DeviceManagers/LidarManager.hpp"
#include "DeviceManagers/MouseManager.hpp"
#include "DeviceManagers/ReflectanceSensorManager.hpp"
#include "DeviceManagers/DriveMotorManager.hpp"
#include "logging/LoggingController.hpp"
#include <bitset>
#include "RobotControl/DeviceManager.hpp"
#include "Utilities/TimeManager.hpp"

#include "Utilities/PID.hpp"
#include "RobotControl/LineFollower.hpp"
#include "RobotControl/AutoReturnToStart.hpp"
#include "Utilities/PathReader.hpp"

using namespace RobotCode::DeviceManagers;
using namespace RobotCode::Logging;
using namespace RobotCode::RobotControl;
using namespace RobotCode::Utilities;

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;

int main(int argc, char* argv[]) {
  std::string path;
  if (argc == 0 || argc == 1) {
    path = "/home/pi/path.csv";
  } else if (argc == 2) {
    path = argv[1];
  } else {
    std::cout << "Invalid arguments" << std::endl;
    return 1;
  }
  PathReader::PathParameters pathParameters;
  PathReader::readPath(path, pathParameters);
  TimeManager::resetStartTime();
  boost::log::sources::channel_logger<> m_logger(keywords::channel = "device");
  m_logger.add_attribute("Device", attrs::constant<std::string>("Main"));
  LoggingController::init(pathParameters.pathName);
  DeviceManager dm;
  dm.startup();
  LineFollower lf(dm.getReflectanceSensorManager(), dm.getDriveTrain(), dm.getGyroManager(), dm.getLidarLogManager());
  try {
    lf.followLine(pathParameters.path, pathParameters.pathRep);
  } catch (std::runtime_error& e) {
    dm.shutdown();
    auto time = std::chrono::system_clock::now();
    long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
        (time - TimeManager::getStartTime()).count();
    BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog)
                        << "Exception caught in main: " << e.what();
    std::cout << "Exception caught in main: " << e.what() << std::endl;
    throw e;
  }
  dm.shutdown();

  return 0;
}
