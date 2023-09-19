#include "DeviceManagers/GyroManager.hpp"
#include <wiringPiI2C.h>
#include <chrono>
#include <thread>

namespace RobotCode::DeviceManagers {

void GyroManager::initialize() {
  if (initialized) {
    throw std::logic_error("GyroManager already initialized.");
  }
  fd = wiringPiI2CSetup(MPU6050_ADDRESS);
  // reset MPU6050
  int pwrMgmt1Val = wiringPiI2CReadReg8(fd, PWR_MGMT_1_REGISTER);
  wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REGISTER, pwrMgmt1Val | 0x80);
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pwrMgmt1Val = wiringPiI2CReadReg8(fd, PWR_MGMT_1_REGISTER);
  } while ((pwrMgmt1Val & 0x80) != 0);
  pwrMgmt1Val = wiringPiI2CReadReg8(fd, PWR_MGMT_1_REGISTER);
  // exit sleep mode
  wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REGISTER, pwrMgmt1Val & 0xBF);
  // set DLPF
  int configVal = wiringPiI2CReadReg8(fd, CONFIG_REGISTER);
  int configValSet;
  if (DLPF_ENABLED) {
    configValSet = (configVal & 0xF8) | 0x03;
  } else {
    configValSet = (configVal & 0xF8);
  }
  wiringPiI2CWriteReg8(fd, CONFIG_REGISTER, configValSet);
  // write SMPLRT_DIV
  wiringPiI2CWriteReg8(fd, SMPLRT_DIV_REGISTER, SMPLRT_DIV);
  initialized = true;
}

void GyroManager::quickStart() {
  initialize();
  startUpdateThread();
}

void GyroManager::startUpdateThread() {
  ensureInitialized();
  if (threadRunning) {
    throw std::logic_error("GyroManager update thread already running.");
  }
  threadInterrupt = false;
  updateThread = std::thread(&GyroManager::update, this);
  threadRunning = true;
}

void GyroManager::stopUpdateThread() {
  ensureInitialized();
  if (!threadRunning) {
    throw std::logic_error("GyroManager update thread not running.");
  }
  threadMutex.lock();
  threadInterrupt = true;
  threadMutex.unlock();
  threadCond.notify_all();
  updateThread.join();
  threadRunning = false;
}

void GyroManager::shutdown() {
  ensureInitialized();
  if (threadRunning) {
    stopUpdateThread();
  }
  // put MPU6050 in sleep mode
  int pwrMgmt1Val = wiringPiI2CReadReg8(fd, PWR_MGMT_1_REGISTER);
  wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REGISTER, pwrMgmt1Val | 0x40);
  initialized = false;
}

void GyroManager::quickStop() {
  shutdown();
}

void GyroManager::update() {
  bool run = true;
  auto startTime = std::chrono::system_clock::now();
  while (run) {
    std::unique_lock<std::mutex> lock(threadMutex);
    threadCond.wait_for(lock, std::chrono::milliseconds(1/SAMPLE_RATE));
    if (threadInterrupt) {
      run = false;
    }
    float gyroXRaw = getGyroXRaw();
    float gyroYRaw = getGyroYRaw();
    float gyroZRaw = getGyroZRaw();
    auto endTime = std::chrono::system_clock::now();
    auto dt = endTime - startTime;
    startTime = endTime;
    float dtSeconds = std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
    gyroX.fetch_add(gyroXRaw * dtSeconds);
    gyroY.fetch_add(gyroYRaw * dtSeconds);
    gyroZ.fetch_add(gyroZRaw * dtSeconds);
  }
}

float GyroManager::getGyroXRaw() const {
  ensureInitialized();
  return (float) readRawData(GYRO_XOUT_H_REGISTER, GYRO_XOUT_L_REGISTER)/131.0f - GYRO_X_OFFSET;
}

float GyroManager::getGyroYRaw() const {
  ensureInitialized();
  return (float) readRawData(GYRO_YOUT_H_REGISTER, GYRO_YOUT_L_REGISTER)/131.0f - GYRO_Y_OFFSET;
}

float GyroManager::getGyroZRaw() const {
  ensureInitialized();
  return (float) readRawData(GYRO_ZOUT_H_REGISTER, GYRO_ZOUT_L_REGISTER)/131.0f - GYRO_Z_OFFSET;
}

float GyroManager::getAccelXRaw() const {
  ensureInitialized();
  return (float) readRawData(ACCEL_XOUT_H_REGISTER, ACCEL_XOUT_L_REGISTER)/16384.0f - ACCEL_X_OFFSET;
}

float GyroManager::getAccelYRaw() const {
  ensureInitialized();
  return (float) readRawData(ACCEL_YOUT_H_REGISTER, ACCEL_YOUT_L_REGISTER)/16384.0f - ACCEL_Y_OFFSET;
}

float GyroManager::getAccelZRaw() const {
  ensureInitialized();
  return (float) readRawData(ACCEL_ZOUT_H_REGISTER, ACCEL_ZOUT_L_REGISTER)/16384.0f - ACCEL_Z_OFFSET;
}

float GyroManager::getGyroX() const {
  ensureInitialized();
  return gyroX;
}

float GyroManager::getGyroY() const {
  ensureInitialized();
  return gyroY;
}

float GyroManager::getGyroZ() const {
  ensureInitialized();
  return gyroZ;
}

short GyroManager::readRawData(int highRegister, int lowRegister) const {
  int highByte, lowByte;
  highByte = wiringPiI2CReadReg8(fd, highRegister);
  lowByte = wiringPiI2CReadReg8(fd, lowRegister);
  return (short)(highByte << 8 | lowByte);
}

void GyroManager::ensureInitialized() const {
  if (!initialized) {
    throw std::logic_error("GyroManager not initialized.");
  }
}

float GyroManager::getTempRaw() const {
  ensureInitialized();
  return readRawData(TEMP_OUT_H_REGISTER, TEMP_OUT_L_REGISTER);
}

float GyroManager::getTemp() const {
  ensureInitialized();
  return getTempRaw() / 340.f + 36.53f;
}

void GyroManager::resetGyroXYZ() {
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;
}

GyroManager::~GyroManager() {
  if (initialized) {
    shutdown();
  }
}

}