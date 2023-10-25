#include "DeviceManagers/ReflectanceSensorManager.hpp"

#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <bitset>
#include <boost/log/attributes/constant.hpp>
#include <boost/log/keywords/channel.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;

using namespace RobotCode::Utilities;

namespace RobotCode::DeviceManagers {

ReflectanceSensorManager::ReflectanceSensorManager(std::chrono::microseconds decayTime) :
  decayTime(decayTime),
  m_logger(keywords::channel = "device") {
    m_logger.add_attribute("Device", attrs::constant<std::string>("ReflectanceSensor"));
}

void ReflectanceSensorManager::initialize() {
  if (initialized) {
    throw std::logic_error("ReflectanceSensorManager already initialized.");
  }
  wiringPiSetupGpio();
  pinMode(LED_ON_GPIO, OUTPUT);
  initialized = true;
}

void ReflectanceSensorManager::ensureInitialized() const {
  if (!initialized) {
    throw std::logic_error("ReflectanceSensorManager not initialized.");
  }
}

void ReflectanceSensorManager::quickStart() {
  initialize();
  startUpdateThread();
}

void ReflectanceSensorManager::quickStop() {
  stopUpdateThread();
}

void ReflectanceSensorManager::startUpdateThread() {
  ensureInitialized();
  if (threadRunning) {
    throw std::logic_error("ReflectanceSensorManager update thread already running.");
  }
  sensorValues = readSenors();
  threadInterrupt = false;
  updateThread = std::thread(&ReflectanceSensorManager::update, this);
  threadRunning = true;
}

void ReflectanceSensorManager::stopUpdateThread() {
  if (!threadRunning) {
    throw std::logic_error("ReflectanceSensorManager update thread not running.");
  }
  threadMutex.lock();
  threadInterrupt = true;
  threadMutex.unlock();
  threadCond.notify_all();
  updateThread.join();
  threadRunning = false;
}

char ReflectanceSensorManager::readSenors() {
  ensureInitialized();
  // 1. turn on IR LEDs
  digitalWrite(LED_ON_GPIO, HIGH);
  // 2. Set IO line to an output and drive it high
  for (int i : SENSOR_GPIO) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  // 3. Allow at least 10 μs for the sensor output to rise
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  // 4. Make the I/O line an input (high impedance)
  auto startTime = std::chrono::system_clock::now();
  for (int i : SENSOR_GPIO) {
    pinMode(i, INPUT);
  }
  // 5. Measure the time for the voltage to decay by waiting for the I/O line to go low
  std::this_thread::sleep_for(decayTime);
  char read = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    read |= (digitalRead(SENSOR_GPIO[i]) << i);
  }
  auto endTime = std::chrono::system_clock::now();
  // 6. turn off IR LEDs
  digitalWrite(LED_ON_GPIO, LOW);
  if (endTime - startTime > 1.6 * decayTime) {
    /*std::cout << "WARNING: Reflectance Sensor decay time " <<
    std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()
    << " μs is greater than decay time limit of "
    << std::chrono::duration_cast<std::chrono::microseconds>(1.6 * decayTime).count() << " μs. "
    << "Using last read value." << std::endl;*/
    return sensorValues;
  } else {
    return read;
  }
}

void ReflectanceSensorManager::update() {
  bool run = true;
  std::unique_lock<std::mutex> lock(threadMutex);
  while (run) {
    threadCond.wait_for(lock, std::chrono::milliseconds(10));
    if (threadInterrupt) {
      run = false;
    }
    sensorValues = readSenors();
    auto timeNow = std::chrono::system_clock::now();
    long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
        (timeNow - TimeManager::getStartTime()).count();
    BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp",
                                              timeLog)
                        << std::bitset<8>(sensorValues);
  }
}

char ReflectanceSensorManager::getSensorValues() {
  ensureInitialized();
  if (!threadRunning) {
    throw std::logic_error("ReflectanceSensorManager update thread not running.");
  }
  return sensorValues;
}

ReflectanceSensorManager::~ReflectanceSensorManager() {
  if (threadRunning) {
    stopUpdateThread();
  }
}

}