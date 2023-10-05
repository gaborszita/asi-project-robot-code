#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
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

using namespace RobotCode::DeviceManagers;
using namespace RobotCode::Logging;
using namespace RobotCode::RobotControl;
using namespace RobotCode::Utilities;

int main() {
  TimeManager::resetStartTime();
  LoggingController::init();
  DeviceManager dm;
  dm.startup();
  LineFollower lf(dm.getReflectanceSensorManager(), dm.getDriveTrain());
  lf.followLine();
  dm.shutdown();

  return 0;
}
