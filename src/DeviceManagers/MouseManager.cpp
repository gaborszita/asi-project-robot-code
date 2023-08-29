#include "DeviceManagers/MouseManager.hpp"
#include <fstream>
#include <iostream>
#include <csignal>

namespace RobotCode::DeviceManagers {

void MouseManager::startMouseThread() {
  if (threadRunning) {
    throw std::logic_error("GyroManager update thread already running.");
  }
  std::thread updateThread = std::thread(&MouseManager::update, this);
  updateThread.detach();
  threadRunning = true;
}

float MouseManager::getX() const {
  return x;
}

float MouseManager::getY() const {
  return y;
}

[[noreturn]] void MouseManager::update() {
  // Open Mouse
  std::ifstream mouseDevice(mouseDevicePath);

  if (!mouseDevice) {
    throw std::runtime_error("Failed to open mouse device " + mouseDevicePath);
  }

  while (true) {
    unsigned char data[3];
    mouseDevice.read(reinterpret_cast<char *>(&data), sizeof(data));

    if (mouseDevice) {
      auto xRead = static_cast<signed char>(data[1]);
      auto yRead = static_cast<signed char>(data[2]);
      x += (float) xRead / MOUSE_DOTS_PER_METER;
      y += (float) yRead / MOUSE_DOTS_PER_METER;
    }
  }
}

const std::string MouseManager::mouseDevicePath = "/dev/input/mouse0";

void MouseManager::quickStart() {
  startMouseThread();
}

}