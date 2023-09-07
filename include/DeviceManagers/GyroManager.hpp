#ifndef ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP
#define ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace RobotCode::DeviceManagers {

class GyroManager {
 public:
  ~GyroManager();
  void initialize();
  void startUpdateThread();
  void quickStart();
  void shutdown();
  void quickStop();
  void stopUpdateThread();
  void resetGyroXYZ();
  [[nodiscard]] float getGyroX() const;
  [[nodiscard]] float getGyroY() const;
  [[nodiscard]] float getGyroZ() const;
  [[nodiscard]] float getGyroXRaw() const;
  [[nodiscard]] float getGyroYRaw() const;
  [[nodiscard]] float getGyroZRaw() const;
  [[nodiscard]] float getAccelXRaw() const;
  [[nodiscard]] float getAccelYRaw() const;
  [[nodiscard]] float getAccelZRaw() const;
  [[nodiscard]] float getTempRaw() const;
  [[nodiscard]] float getTemp() const;

 private:
  [[nodiscard]] short readRawData(int highRegister, int lowRegister) const;
  void update();
  void ensureInitialized() const;

  int fd;

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

  constexpr static const float GYRO_X_OFFSET = -2.56;
  constexpr static const float GYRO_Y_OFFSET = +1.02;
  constexpr static const float GYRO_Z_OFFSET = +0.225;
  constexpr static const float ACCEL_X_OFFSET = +0.066;
  constexpr static const float ACCEL_Y_OFFSET = +0.072;
  constexpr static const float ACCEL_Z_OFFSET = +0.96;

  std::atomic<float> gyroX = 0;
  std::atomic<float> gyroY = 0;
  std::atomic<float> gyroZ = 0;

  std::thread updateThread;
  bool threadRunning = false;
  bool threadInterrupt = false;
  std::mutex threadMutex;
  std::condition_variable threadCond;

  bool initialized = false;
};

}

#endif //ROBOT_CODE_DEVICEMANAGERS_GYROMANAGER_HPP
