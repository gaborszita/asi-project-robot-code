#include <iostream>

#include <rplidar.h>
#include "Adafruit_MotorHAT.h"

Adafruit_MotorHAT hat;

int main() {
  std::cout << "Hello, World!" << std::endl;
  rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
  return 0;
}
