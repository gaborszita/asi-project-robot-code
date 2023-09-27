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

Adafruit_MotorHAT hat;

using namespace RobotCode::DeviceManagers;
using namespace RobotCode::Logging;

namespace logging = boost::log;
namespace keywords = boost::log::keywords;

int main() {
  std::cout << "Hello, World!" << std::endl;
  //rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();

  /*GyroManager mg;
  mg.initialize();
  while (1) {
    std::cout << mg.getXAngle() << std::endl;
  }*/

  /*LidarManager lm;
  lm.quickStart();
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
  lm.grabScanData(nodes, nodeCount);
  std::cout << "size: " << nodeCount << std::endl;
  for (int i=0; i<nodeCount; i++) {
    auto node = nodes[i];
    float angle_in_degrees = node.angle_z_q14 * 90.f / (1 << 14);
    float distance_in_meters = node.dist_mm_q2 / 1000.f / (1 << 2);
    std::cout << angle_in_degrees << " " << distance_in_meters << std::endl;
  }
  lm.quickStop();*/

  LoggingController::init();
  /*GyroManager gm;
  gm.quickStart();
  double gyroxsum, gyroysum, gyrozsum, accelxsum, accelysum, accelzsum;
  //while (true) {
    //std::cout << "gyro x: " << gm.getGyroX() << std::endl;
    //std::cout << "gyro y: " << gm.getGyroY() << std::endl;
    //std::cout << "gyro z: " << gm.getGyroZ() << std::endl;
    sleep(1);
  //}
  gm.quickStop();*/

  /*MouseManager mg;
  //mg.quickStart();
  int i=0;
  auto targetTimeNano = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(1)).count();
  PID pid(0.1, 0.0, -targetTimeNano, 0, 0.0, 1);
  bool firstRun = true;
  long long lastNano;
  long long waitTimeNano;
  while (i<1000000000) {
    //std::cout << mg.getX() << " " << mg.getY() << std::endl;
    sleep(0.0001);
      if (!firstRun) {
          std::cout << "last nano: " << (double) lastNano << std::endl;
          std::cout << "target nano: " << (double) targetTimeNano << std::endl;
          std::cout << "pid out: " << pid.calculate(targetTimeNano, lastNano) << std ::endl;
          waitTimeNano = targetTimeNano + pid.calculate(targetTimeNano, lastNano);
          std::cout << "wait time: " << waitTimeNano << std::endl;
      } else {
          waitTimeNano = targetTimeNano;
      }
    auto start = std::chrono::high_resolution_clock::now();


    std::this_thread::sleep_for(std::chrono::nanoseconds ((long long) waitTimeNano));
      //auto between = std::chrono::high_resolution_clock::now();

      auto finish = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() << "ns\n";
    lastNano = std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count();
    firstRun = false;
    i++;
  }*/

  /*ReflectanceSensorManager rsm(std::chrono::microseconds(100));
  rsm.quickStart();
  rsm.quickStop();*/

  /*while (1) {
    //sleep(1);
    char val = rsm.getSensorValues();
    std::cout << std::bitset<8>(val) << std::endl;
  }*/

  /*DriveMotorManager dgm;
  dgm.runMotor(DriveMotorManager::BackRight, DriveMotorManager::Forward, 100);
  sleep(2);
  dgm.releaseMotor(DriveMotorManager::BackRight);
  dgm.runMotor(DriveMotorManager::BackLeft, DriveMotorManager::Forward, 100);
  sleep(2);
  dgm.releaseMotor(DriveMotorManager::BackLeft);
  dgm.runMotor(DriveMotorManager::FrontLeft, DriveMotorManager::Forward, 100);
  sleep(2);
  dgm.releaseMotor(DriveMotorManager::FrontLeft);
  dgm.runMotor(DriveMotorManager::FrontRight, DriveMotorManager::Forward, 100);
  sleep(2);
  dgm.releaseMotor(DriveMotorManager::FrontRight);*/



    //std::ofstream out("/home/pi/sampletestcreate.log");
    //out << "Hello, World!";
    //out.close();




  //boost::log::add_file_log(boost::log::keywords::target = "/home/pi",
  //                         boost::log::keywords::file_name = "%y%m%d_%3N.log");
  // wait for 10 seconds
  //std::this_thread::sleep_for(std::chrono::seconds(10));

  /*boost::log::core::get()->set_filter
      (
          boost::log::trivial::severity >= boost::log::trivial::info
      );

  boost::log::add_common_attributes();

  BOOST_LOG_TRIVIAL(info) << "Hello, World!";
  BOOST_LOG_TRIVIAL(fatal) << "Hello, World!";*/


  /*GyroManager gm;
  ReflectanceSensorManager rsm(std::chrono::microseconds(100));
  MouseManager mm;
  DriveMotorManager dmm;
  gm.quickStart();
  rsm.quickStart();
  mm.quickStart();
  sleep(1);
  dmm.runMotor(DriveMotorManager::FrontLeft, DriveMotorManager::Forward, 100);
  sleep(1);
  gm.quickStop();
  rsm.quickStop();
  dmm.releaseMotor(DriveMotorManager::FrontLeft);*/

  RobotCode::Utilities::TimeManager::resetStartTime();
  RobotCode::RobotControl::DeviceManager dm;
  dm.startup();
  RobotCode::RobotControl::LineFollower lf(dm.getReflectanceSensorManager(), dm.getDriveTrain());
  lf.followLine();
  //dm.getDriveTrain().drive(RobotCode::RobotControl::DriveTrain::Direction::TurnLeft, RobotCode::RobotControl::DriveTrain::Speed::Fast);
  //sleep(70);
  //dm.getDriveTrain().stop();
  dm.shutdown();

  return 0;
}
