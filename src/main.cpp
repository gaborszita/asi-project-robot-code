#include <iostream>
#include <unistd.h>

#include "rplidar.h"
#include "Adafruit_MotorHAT.h"
#include "DeviceManagers/GyroManager.hpp"
#include "DeviceManagers/LidarManager.hpp"
#include "DeviceManagers/MouseManager.hpp"

Adafruit_MotorHAT hat;

using namespace RobotCode::DeviceManagers;

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

  /*GyroManager gm;
  gm.quickStart();
  double gyroxsum, gyroysum, gyrozsum, accelxsum, accelysum, accelzsum;
  for(int i=0; i<6; i++) {
    std::cout << "gyro x: " << gm.getGyroX() << std::endl;
    std::cout << "gyro y: " << gm.getGyroY() << std::endl;
    std::cout << "gyro z: " << gm.getGyroZ() << std::endl;
    sleep(1);
  }
  gm.quickStop();*/

  MouseManager mg;
  mg.quickStart();
  int i=0;
  while (i<1000) {
    std::cout << mg.getX() << " " << mg.getY() << std::endl;
    sleep(1);
    i++;
  }

  return 0;
}
