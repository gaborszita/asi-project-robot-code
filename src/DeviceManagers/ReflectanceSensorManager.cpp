#include "DeviceManagers/ReflectanceSensorManager.hpp"

#include <wiringPi.h>
#include <iostream>
#include <thread>

namespace RobotCode::DeviceManagers {

ReflectanceSensorManager::ReflectanceSensorManager(std::chrono::microseconds decayTime) : decayTime(decayTime) {
}

void ReflectanceSensorManager::initialize() {
  if (initialized) {
    throw std::logic_error("GyroManager already initialized.");
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
  for (int i : SENSOR_GPIO) {
    pinMode(i, INPUT);
  }
  // 5. Measure the time for the voltage to decay by waiting for the I/O line to go low
  std::this_thread::sleep_for(decayTime);
  char read = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    read |= (digitalRead(SENSOR_GPIO[i]) << i);
  }
  // 6. turn off IR LEDs
  digitalWrite(LED_ON_GPIO, LOW);
  return read;
}

void ReflectanceSensorManager::update() {
  bool run = true;
  while (run) {
    std::unique_lock<std::mutex> lock(threadMutex);
    threadCond.wait_for(lock, std::chrono::milliseconds(10));
    if (threadInterrupt) {
      run = false;
    }
    sensorValues = readSenors();
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