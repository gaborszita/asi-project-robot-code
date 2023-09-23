#include "DeviceManagers/MouseManager.hpp"
#include <fstream>
#include <iostream>
#include <csignal>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/attributes/constant.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

using namespace RobotCode::Utilities;

namespace RobotCode::DeviceManagers {

MouseManager::MouseManager() :
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("Mouse"));
}

void MouseManager::startMouseThread() {
  if (threadRunning) {
    throw std::logic_error("MouseManager update thread already running.");
  }
  std::thread updateThread = std::thread(&MouseManager::update, this);
  updateThread.detach();
  threadRunning = true;
}

void MouseManager::stopMouseThread() {
  if (!threadRunning) {
    throw std::logic_error("MouseManager update thread not running.");
  } else if (threadStopQueued) {
    throw std::logic_error("MouseManager update thread already queued stopped.");
  }
  threadMutex.lock();
  threadInterrupt = true;
  threadMutex.unlock();
  threadStopQueued = true;
}

float MouseManager::getX() const {
  return x;
}

float MouseManager::getY() const {
  return y;
}

void MouseManager::update() {
  // Open Mouse
  std::ifstream mouseDevice(mouseDevicePath);

  if (!mouseDevice) {
    throw std::runtime_error("Failed to open mouse device " + mouseDevicePath);
  }

  std::unique_lock<std::mutex> lock(threadMutex);
  while (true) {
    unsigned char data[3];
    lock.unlock();
    mouseDevice.read(reinterpret_cast<char *>(&data), sizeof(data));
    lock.lock();
    if (threadInterrupt) {
      break;
    }

    if (mouseDevice) {
      auto xRead = static_cast<signed char>(data[1]);
      auto yRead = static_cast<signed char>(data[2]);
      std::atomic<long> xff{};
      x.fetch_add((float) xRead / MOUSE_DOTS_PER_METER);
      y.fetch_add((float) yRead / MOUSE_DOTS_PER_METER);
    }

    auto timeNow = std::chrono::system_clock::now();
    long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
        (timeNow - TimeManager::getStartTime()).count();
    BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp",
                                              timeLog)
                        << x << "," << y;
  }
}

const std::string MouseManager::mouseDevicePath = "/dev/input/mouse0";

void MouseManager::quickStart() {
  startMouseThread();
}

void MouseManager::quickStop() {
  stopMouseThread();
}

}