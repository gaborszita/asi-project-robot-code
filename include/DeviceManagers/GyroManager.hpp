#ifndef ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <boost/log/sources/channel_logger.hpp>

namespace RobotCode::DeviceManagers {

class GyroManager {
 public:
  GyroManager();
  ~GyroManager();
  void initialize();
  void startUpdateThread();
  void quickStart();
  void shutdown();
  void quickStop();
  void stopUpdateThread();
  void resetGyroXYZ();
  [[nodiscard]] double getGyroX() const;
  [[nodiscard]] double getGyroY() const;
  [[nodiscard]] double getGyroZ() const;
  [[nodiscard]] double getGyroXRaw() const;
  [[nodiscard]] double getGyroYRaw() const;
  [[nodiscard]] double getGyroZRaw() const;
  [[nodiscard]] double getAccelXRaw() const;
  [[nodiscard]] double getAccelYRaw() const;
  [[nodiscard]] double getAccelZRaw() const;
  [[nodiscard]] double getTempRaw() const;
  [[nodiscard]] double getTemp() const;

 private:
  [[nodiscard]] short readRawData(int highRegister, int lowRegister) const;
  void update();
  void ensureInitialized() const;
  void logData(std::chrono::time_point<std::chrono::system_clock> time);

  int fd{};

  static const int MPU6050_ADDRESS = 0x68;
  static const int PWR_MGMT_1_REGISTER = 0x6B;
  static const int CONFIG_REGISTER = 0x1A;
  static const int SMPLRT_DIV_REGISTER = 0x19;
  static const int ACCEL_XOUT_H_REGISTER = 0x3B;
  static const int ACCEL_XOUT_L_REGISTER = 0x3C;
  static const int ACCEL_YOUT_H_REGISTER = 0x3D;
  static const int ACCEL_YOUT_L_REGISTER = 0x3E;
  static const int ACCEL_ZOUT_H_REGISTER = 0x3F;
  static const int ACCEL_ZOUT_L_REGISTER = 0x40;
  static const int TEMP_OUT_H_REGISTER = 0x41;
  static const int TEMP_OUT_L_REGISTER = 0x42;
  static const int GYRO_XOUT_H_REGISTER = 0x43;
  static const int GYRO_XOUT_L_REGISTER = 0x44;
  static const int GYRO_YOUT_H_REGISTER = 0x45;
  static const int GYRO_YOUT_L_REGISTER = 0x46;
  static const int GYRO_ZOUT_H_REGISTER = 0x47;
  static const int GYRO_ZOUT_L_REGISTER = 0x48;

  static const bool DLPF_ENABLED = true;
  static const int SMPLRT_DIV = 0x03;
  static const int SAMPLE_RATE = (DLPF_ENABLED ? 1000 : 8000) / (1 + SMPLRT_DIV);

  constexpr static const double GYRO_X_OFFSET = -2.56;
  constexpr static const double GYRO_Y_OFFSET = +1.02;
  constexpr static const double GYRO_Z_OFFSET = +0.16;
  constexpr static const double ACCEL_X_OFFSET = +0.066;
  constexpr static const double ACCEL_Y_OFFSET = +0.072;
  constexpr static const double ACCEL_Z_OFFSET = +0.96;

  std::atomic<double> gyroX = 0;
  std::atomic<double> gyroY = 0;
  std::atomic<double> gyroZ = 0;

  std::thread updateThread;
  bool threadRunning = false;
  bool threadInterrupt = false;
  std::mutex threadMutex;
  std::condition_variable threadCond;

  bool initialized = false;

  boost::log::sources::channel_logger<> m_logger;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP
