#include "DeviceManagers/LidarManager.hpp"
#include <iostream>

using namespace rp::standalone::rplidar;

namespace RobotCode::DeviceManagers {

void LidarManager::initialize() {
  if (lidar == nullptr) {
    lidar = RPlidarDriver::CreateDriver();
  }
}

void LidarManager::shutdown() {
  if (lidar != nullptr) {
    RPlidarDriver::DisposeDriver(lidar);
    lidar = nullptr;
  }
}

void LidarManager::connect() {
  if (lidar == nullptr) {
    initialize();
  } else if (lidar->isConnected()) {
    throw std::logic_error("Lidar already connected.");
  }

  u_result res = lidar->connect("/dev/ttyUSB0", 115200);
  if (!IS_OK(res)) {
    throw std::runtime_error("Failed to connect to lidar: " + std::to_string(res));
  }
  scanStarted = false;
}

void LidarManager::disconnect() {
  if (lidar == nullptr) {
    throw std::logic_error("LidarManager not initialized.");
  } else if (!lidar->isConnected()) {
    throw std::logic_error("Lidar already disconnected.");
  } else {
    lidar->disconnect();
  }
}

LidarManager::~LidarManager() {
  shutdown();
}

void LidarManager::startScan() {
  RplidarScanMode scanMode{};
  lidar->startScan(false, true, 0, &scanMode);

  scanStarted = true;
}

void LidarManager::grabScanData(rplidar_response_measurement_node_hq_t *nodes, size_t &nodeCount) {
  if (!scanStarted) {
    startScan();
  }

  u_result res = lidar->grabScanDataHq(nodes, nodeCount);

  if (IS_FAIL(res))
  {
    throw std::runtime_error("Failed to get scan data");
  }
}

void LidarManager::startMotor() {
  if (lidar == nullptr) {
    throw std::logic_error("LidarManager not initialized.");
  } else if (!lidar->isConnected()) {
    throw std::logic_error("Lidar not connected.");
  } else {
    lidar->startMotor();
  }
}

void LidarManager::stopMotor() {
  if (lidar == nullptr) {
    throw std::logic_error("LidarManager not initialized.");
  } else if (!lidar->isConnected()) {
    throw std::logic_error("Lidar not connected.");
  } else {
    lidar->stopMotor();
  }
}

void LidarManager::quickStart() {
  initialize();
  connect();
  startMotor();
  startScan();
}

void LidarManager::quickStop() {
  stopMotor();
  disconnect();
  shutdown();
}

}