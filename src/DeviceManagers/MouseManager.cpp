#include "DeviceManagers/MouseManager.hpp"
#include <fstream>
#include <iostream>

namespace RobotCode::DeviceManagers {

void MouseManager::startMouseThread() {
  if (threadRunning) {
    throw std::logic_error("GyroManager update thread already running.");
  }
  threadInterrupt = false;
  updateThread = std::thread(&MouseManager::update, this);
  threadRunning = true;
}

void MouseManager::stopMouseThread() {
  if (!threadRunning) {
    throw std::logic_error("GyroManager update thread not running.");
  }
  threadInterrupt = true;
  threadCond.notify_all();
  updateThread.join();
  threadRunning = false;
}

float MouseManager::getX() const {
  return x;
}

float MouseManager::getY() const {
  return y;
}

void MouseManager::update() {
  bool run = true;
  // Open Mouse
  std::ifstream mouseDevice(mouseDevicePath);

  if (!mouseDevice) {
    throw std::runtime_error("Failed to open mouse device " + mouseDevicePath);
  }

  while (run) {
    //std::unique_lock<std::mutex> lock(threadMutex);
    //threadCond.wait(lock);
    if (threadInterrupt) {
      run = false;
    }
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

void MouseManager::quickStop() {
  stopMouseThread();
}

MouseManager::~MouseManager() {
  if (threadRunning) {
    stopMouseThread();
  }
}

}