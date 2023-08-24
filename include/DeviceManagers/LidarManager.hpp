#ifndef ROBOT_CODE_DEVICEMANAGERS_LIDARMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_LIDARMANAGER_HPP

#include <rplidar.h>

namespace RobotCode::DeviceManagers {

class LidarManager {
 public:
  ~LidarManager();
  void initialize();
  void shutdown();
  void connect();
  void disconnect();
  void startMotor();
  void stopMotor();
  void quickStart();
  void quickStop();
  void grabScanData(rplidar_response_measurement_node_hq_t *nodes, size_t &nodeCount);
  void startScan();
 private:
  rp::standalone::rplidar::RPlidarDriver* lidar = nullptr;
  bool scanStarted = false;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_LIDARMANAGER_HPP
