#include "RobotControl/LineFollower.hpp"
#include <iostream>
#include <bitset>

namespace RobotCode::RobotControl {

LineFollower::LineFollower(RobotCode::DeviceManagers::ReflectanceSensorManager &rsm, DriveTrain &driveTrain) :
    rsm(rsm),
    driveTrain(driveTrain) {

}

void LineFollower::followLine() {
  int invalidDataIter = 0;
  int lastDir = 0;
  int remainingWaitBacking = 0;
  while(true) {
    /*
     * 00000000 -> stop, ERROR
00011000 -> center, continue straight
  Num & 0x18 == 0x18
00001xxx -> turn left
xxx10000 -> turn right
11111111 -> stop, end
Everything else -> invalid data error
     */

    char data = rsm.getSensorValues();
    //while (true) {
    //  char data = rsm.getSensorValues();
    //  std::cout << std::bitset<8>(data) << std::endl;
    //}
    std::cout << std::bitset<8>(data) << std::endl;

    if (data == 0x00) {
      if (lastDir == 0) {
        std::cout << "ERROR: No line on reflectance sensor" << std::endl;
        driveTrain.stop();
        return;
      } else if (lastDir == 1) {
        std::cout << "Back turn left" << std::endl;
        driveTrain.drive(DriveTrain::Direction::BackTurnLeft, DriveTrain::Speed::Slow);
      } else {
        std::cout << "Back turn right" << std::endl;
        driveTrain.drive(DriveTrain::Direction::BackTurnRight, DriveTrain::Speed::Slow);
      }
      remainingWaitBacking = 10;
      //invalidDataIter = 0;
      //std::cout << "ERROR: No line on reflectance sensor" << std::endl;
    } else if (data == 0xFF && remainingWaitBacking == 0) {
      invalidDataIter = 0;
      std::cout << "End" << std::endl;
      driveTrain.stop();
      return;
    } else if ((data & 0x18) == 0x18) {
      invalidDataIter = 0;
      remainingWaitBacking > 0 ? remainingWaitBacking-- : remainingWaitBacking = 0;
      std::cout << "Center, continue straight" << std::endl;
      driveTrain.drive(DriveTrain::Direction::Forward, DriveTrain::Speed::Slow);
    } else if ((data & 0x0F) != 0x00 && (data & 0xF0) == 0x00) {
      invalidDataIter = 0;
      remainingWaitBacking > 0 ? remainingWaitBacking-- : remainingWaitBacking = 0;
      if ((data & 0x0F) > 0x03) {
        std::cout << "Turn left" << std::endl;
        driveTrain.drive(DriveTrain::Direction::TurnLeft, DriveTrain::Speed::Slow);
      } else {
        std::cout << "Rotate left" << std::endl;
        driveTrain.drive(DriveTrain::Direction::RotateLeft, DriveTrain::Speed::Slow);
      }
      lastDir = 1;
    } else if ((data & 0x0F) == 0x00 && (data & 0xF0) != 0x00) {
      invalidDataIter = 0;
      remainingWaitBacking > 0 ? remainingWaitBacking-- : remainingWaitBacking = 0;
      if ((data & 0xF0) > 0x30) {
        std::cout << "Rotate right" << std::endl;
        driveTrain.drive(DriveTrain::Direction::RotateRight, DriveTrain::Speed::Slow);
      } else {
        std::cout << "Turn right" << std::endl;
        driveTrain.drive(DriveTrain::Direction::TurnRight, DriveTrain::Speed::Slow);
      }
      lastDir = 2;
      //std::cout << "Turn right" << std::endl;
      //driveTrain.drive(DriveTrain::Direction::TurnRight, DriveTrain::Speed::Slow);
    }  else {
      lastDir = 0;
      // stpop, then iterate five times waiting 100 ms between each to give a chance for a good data
      // if still invalid, then error
      if (invalidDataIter < 5) {
        std::cout << "Invalid data from reflectance sensor " << std::bitset<8>(data) << std::endl;
        driveTrain.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        invalidDataIter++;
      } else {
        std::cout << "ERROR: Invalid data from reflectance sensor " << std::bitset<8>(data) << std::endl;
        driveTrain.stop();
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}