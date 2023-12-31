#include "DeviceManagers/GyroManager.hpp"
#include <wiringPiI2C.h>
#include <chrono>
#include <thread>
#include <boost/log/attributes.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/keywords/channel.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;

using namespace RobotCode::Utilities;

namespace RobotCode::DeviceManagers {

GyroManager::GyroManager() :
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("Gyro"));
}

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

void GyroManager::logData(std::chrono::time_point<std::chrono::system_clock> time) {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (time - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      getGyroX() << "," << getGyroY() << "," << getGyroZ() << "," <<
                      getGyroXRaw() << "," << getGyroYRaw() << "," << getGyroZRaw() << "," <<
                      getAccelXRaw() << "," << getAccelYRaw() << "," << getAccelZRaw() << "," <<
                      getTempRaw() << "," << getTemp();
}

void GyroManager::update() {
  bool run = true;
  auto startTime = std::chrono::system_clock::now();
  std::unique_lock<std::mutex> lock(threadMutex);
  while (run) {
    threadCond.wait_for(lock, std::chrono::milliseconds(1/SAMPLE_RATE));
    if (threadInterrupt) {
      run = false;
    }
    double gyroXRaw = getGyroXRaw();
    double gyroYRaw = getGyroYRaw();
    double gyroZRaw = getGyroZRaw();
    auto endTime = std::chrono::system_clock::now();
    auto dt = endTime - startTime;
    startTime = endTime;
    double dtSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
    gyroX.fetch_add(gyroXRaw * dtSeconds);
    gyroY.fetch_add(gyroYRaw * dtSeconds);
    gyroZ.fetch_add(gyroZRaw * dtSeconds);
    logData(endTime);
  }
}

double GyroManager::getGyroXRaw() const {
  ensureInitialized();
  return (double) readRawData(GYRO_XOUT_H_REGISTER, GYRO_XOUT_L_REGISTER)/131.0 - GYRO_X_OFFSET;
}

double GyroManager::getGyroYRaw() const {
  ensureInitialized();
  return (double) readRawData(GYRO_YOUT_H_REGISTER, GYRO_YOUT_L_REGISTER)/131.0 - GYRO_Y_OFFSET;
}

double GyroManager::getGyroZRaw() const {
  ensureInitialized();
  return (double) readRawData(GYRO_ZOUT_H_REGISTER, GYRO_ZOUT_L_REGISTER)/131.0 - GYRO_Z_OFFSET;
}

double GyroManager::getAccelXRaw() const {
  ensureInitialized();
  return (double) readRawData(ACCEL_XOUT_H_REGISTER, ACCEL_XOUT_L_REGISTER)/16384.0 - ACCEL_X_OFFSET;
}

double GyroManager::getAccelYRaw() const {
  ensureInitialized();
  return (double) readRawData(ACCEL_YOUT_H_REGISTER, ACCEL_YOUT_L_REGISTER)/16384.0 - ACCEL_Y_OFFSET;
}

double GyroManager::getAccelZRaw() const {
  ensureInitialized();
  return (double) readRawData(ACCEL_ZOUT_H_REGISTER, ACCEL_ZOUT_L_REGISTER)/16384.0 - ACCEL_Z_OFFSET;
}

double GyroManager::getGyroX() const {
  ensureInitialized();
  return gyroX;
}

double GyroManager::getGyroY() const {
  ensureInitialized();
  return gyroY;
}

double GyroManager::getGyroZ() const {
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

double GyroManager::getTempRaw() const {
  ensureInitialized();
  return readRawData(TEMP_OUT_H_REGISTER, TEMP_OUT_L_REGISTER);
}

double GyroManager::getTemp() const {
  ensureInitialized();
  return getTempRaw() / 340 + 36.53;
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