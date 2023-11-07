#include "RobotControl/AutoReturnToStart.hpp"
#include "RobotControl/LineFollowerFSMAutoReturnToStart.hpp"
#include "logging/LidarLogManager.hpp"
#include <iostream>
#include <limits>
#include <boost/log/attributes.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/manipulators.hpp>
#include <boost/log/keywords/channel.hpp>
#include "Utilities/TimeManager.hpp"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;

using namespace RobotCode::Logging;
using namespace RobotCode::DeviceManagers;
using namespace RobotCode::RobotControl::LineFollowerFSMAutoReturnToStart;
using namespace RobotCode::Utilities;

namespace RobotCode::RobotControl {

AutoReturnToStart::AutoReturnToStart(LidarLogManager &lidarLogManager,
                                     DriveTrain &driveTrain,
                                     GyroManager &gyroManager,
                                     ReflectanceSensorManager& rsm) :
    lidarLogManager(lidarLogManager),
    driveTrain(driveTrain),
    gyroManager(gyroManager),
    reflectanceSensorManager(rsm),
    m_logger(keywords::channel = "device") {
  m_logger.add_attribute("Device", attrs::constant<std::string>("AutoReturnToStart"));
}

void AutoReturnToStart::returnToStart() {
  std::cout << "Auto return to start" << std::endl;
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Auto return to start";
  bool returnedToStart = false;
  while (!returnedToStart) {
    ensureStartDistance();
    rotate();
    driveToFrdBwdMiddle();
    rotate();
    ensureRgtLftCorrectDistance();
    returnedToStart = strideToPath() && goToStart() && verifyStart();
  }
}

AutoReturnToStart::LidarReadingStatus AutoReturnToStart::checkPoint(rplidar_response_measurement_node_hq_t *nodes,
                                                                    size_t nodeCount) {
  LidarReadingStatus status{};
  status.rightTooClose = false;
  status.leftTooClose = false;
  status.frontTooClose = false;
  status.backTooClose = false;
  int consecutiveCount = 0;
  int consecutiveInvalidCount = 0;
  int tooCloseDir = -1;
  float consecutiveStartAngle;
  float consecutiveInvalidStartAngle;
  float frontAverage = 0;
  int frontCnt = 0;
  float backAverage = 0;
  int backCnt = 0;
  float rightAverage = 0;
  int rightCnt = 0;
  float leftAverage = 0;
  int leftCnt = 0;
  for (int i = 0; i < nodeCount; i++) {
    float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
    float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
    if (angle_in_degrees >= 350 && angle_in_degrees <= 10 && nodes[i].quality > 0) {
      frontAverage += distance_in_meters;
      frontCnt++;
    } else if (angle_in_degrees >= 170 && angle_in_degrees <= 190 && nodes[i].quality > 0) {
      backAverage += distance_in_meters;
      backCnt++;
    } else if (angle_in_degrees >= 260 && angle_in_degrees <= 280 && nodes[i].quality > 0) {
      rightAverage += distance_in_meters;
      rightCnt++;
    } else if (angle_in_degrees >= 80 && angle_in_degrees <= 100 && nodes[i].quality > 0) {
      leftAverage += distance_in_meters;
      leftCnt++;
    }
    if (((((angle_in_degrees >= 0 && angle_in_degrees <= 10) ||
        (angle_in_degrees < 360 && angle_in_degrees >= 350)) &&
        distance_in_meters < 0.35)
        || distance_in_meters < 0.25)
        && nodes[i].quality > 0) {
      consecutiveStartAngle = angle_in_degrees;
      consecutiveCount++;
    } else {
      consecutiveCount = 0;
    }
    if (nodes[i].quality == 0) {
      consecutiveInvalidStartAngle = angle_in_degrees;
      consecutiveInvalidCount++;
    } else {
      consecutiveInvalidCount = 0;
    }
    if (consecutiveCount >= 10) {
      if ((consecutiveStartAngle >= 0 && consecutiveStartAngle <= 10) ||
          (consecutiveStartAngle < 360 && consecutiveStartAngle >= 350) ||
          (angle_in_degrees >= 0 && angle_in_degrees <= 10) ||
          (angle_in_degrees < 360 && angle_in_degrees >= 350)) {
        status.backTooClose = true;
      }
      if ((consecutiveStartAngle >= 170 && consecutiveStartAngle <= 190) ||
          (angle_in_degrees >= 170 && angle_in_degrees <= 190)) {
        status.frontTooClose = true;
      }
      if ((consecutiveStartAngle >= 260 && consecutiveStartAngle <= 280) ||
          (angle_in_degrees >= 260 && angle_in_degrees <= 280)) {
        //std::cout << "righttooclose" << std::endl;
        status.rightTooClose = true;
      }
      if ((consecutiveStartAngle >= 80 && consecutiveStartAngle <= 100) ||
          (angle_in_degrees >= 80 && angle_in_degrees <= 100)) {
        status.leftTooClose = true;
      }
    }
    if (consecutiveInvalidCount >= 10) {
      if ((consecutiveStartAngle >= 0 && consecutiveStartAngle <= 10) ||
          (consecutiveStartAngle < 360 && consecutiveStartAngle >= 350) ||
          (angle_in_degrees >= 0 && angle_in_degrees <= 10) ||
          (angle_in_degrees < 360 && angle_in_degrees >= 350)) {
        status.backInvalid = true;
      }
      if ((consecutiveStartAngle >= 170 && consecutiveStartAngle <= 190) ||
          (angle_in_degrees >= 170 && angle_in_degrees <= 190)) {
        status.frontInvalid = true;
      }
      if ((consecutiveStartAngle >= 260 && consecutiveStartAngle <= 280) ||
          (angle_in_degrees >= 260 && angle_in_degrees <= 280)) {
        //std::cout << "righttooclose" << std::endl;
        status.rightInvalid = true;
      }
      if ((consecutiveStartAngle >= 80 && consecutiveStartAngle <= 100) ||
          (angle_in_degrees >= 80 && angle_in_degrees <= 100)) {
        status.leftInvalid = true;
      }
    }
  }

  if (frontCnt > 0) {
    frontAverage /= (float) frontCnt;
  } else {
    frontAverage = std::numeric_limits<float>::max();;
  }
  if (backCnt > 0) {
    backAverage /= (float) backCnt;
  } else {
    backAverage = std::numeric_limits<float>::max();;
  }
  if (frontAverage < backAverage) {
    status.frontCloser = true;
  } else {
    status.frontCloser = false;
  }

  if (rightCnt > 0) {
    rightAverage /= (float) rightCnt;
  } else {
    rightAverage = std::numeric_limits<float>::max();;
  }
  if (leftCnt > 0) {
    leftAverage /= (float) leftCnt;
  } else {
    leftAverage = std::numeric_limits<float>::max();;
  }
  if (rightAverage < leftAverage) {
    status.rightCloser = true;
  } else {
    status.rightCloser = false;
  }

  return status;
}

void AutoReturnToStart::ensureStartDistance() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Ensure start distance";
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);

  LidarReadingStatus status{};
  int allClearCnt = 0;
  bool attemptedFrwBck = false;
  bool attemptedRgtLft = false;
  do {
    lidarLogManager.getScanData(nodes, nodeCount);
    status = checkPoint(nodes, nodeCount);

    //std::cout << "fronttooclose: " << status.frontTooClose << std::endl;
    //std::cout << "backtooclose: " << status.backTooClose << std::endl;
    //std::cout << "righttooclose: " << status.rightTooClose << std::endl;
    //std::cout << "lefttooclose: " << status.leftTooClose << std::endl;
    if (status.frontTooClose || status.backTooClose || status.rightTooClose || status.leftTooClose) {
      allClearCnt = 0;
    }

    if (status.frontTooClose && status.backTooClose) {
      //std::cout << "bothfrontbacktooclose" << std::endl;
      if (status.rightCloser) {
        driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
      } else {
        driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
      }
      attemptedRgtLft = true;
    } else if (status.frontTooClose) {
      //std::cout << "backing" << std::endl;
      driveTrain.drive(DriveTrain::Backward, DriveTrain::Fast);
      attemptedFrwBck = true;
    } else if (status.backTooClose) {
      //std::cout << "forwarding" << std::endl;
      driveTrain.drive(DriveTrain::Forward, DriveTrain::Fast);
      attemptedFrwBck = true;
    } else if (status.rightTooClose && status.leftTooClose) {
      //std::cout << "sideways" << std::endl;
      if (status.frontCloser) {
        driveTrain.drive(DriveTrain::Backward, DriveTrain::Fast);
      } else {
        driveTrain.drive(DriveTrain::Forward, DriveTrain::Fast);
      }
      attemptedFrwBck = true;
    } else if (status.rightTooClose) {
      //std::cout << "striding" << std::endl;
      driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
      attemptedRgtLft = true;
    } else if (status.leftTooClose) {
      //std::cout << "striding" << std::endl;
      driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
      attemptedRgtLft = true;
    } else if (!attemptedFrwBck && status.frontInvalid) {
      driveTrain.drive(DriveTrain::Backward, DriveTrain::Fast);
      auto startTime = std::chrono::system_clock::now();
      while (std::chrono::system_clock::now() - startTime < std::chrono::milliseconds(1000)) {
        lidarLogManager.getScanData(nodes, nodeCount);
        status = checkPoint(nodes, nodeCount);
        if (!status.frontInvalid) {
          break;
        }
      }
      driveTrain.stop();
      attemptedFrwBck = true;
    } else if (!attemptedFrwBck && status.backInvalid) {
      driveTrain.drive(DriveTrain::Backward, DriveTrain::Fast);
      auto startTime = std::chrono::system_clock::now();
      while (std::chrono::system_clock::now() - startTime < std::chrono::milliseconds(1000)) {
        lidarLogManager.getScanData(nodes, nodeCount);
        status = checkPoint(nodes, nodeCount);
        if (!status.backInvalid) {
          break;
        }
      }
      attemptedFrwBck = true;
    } else if (!attemptedRgtLft && status.rightInvalid) {
      driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
      auto startTime = std::chrono::system_clock::now();
      while (std::chrono::system_clock::now() - startTime < std::chrono::milliseconds(1000)) {
        lidarLogManager.getScanData(nodes, nodeCount);
        status = checkPoint(nodes, nodeCount);
        if (!status.rightInvalid) {
          break;
        }
      }
      attemptedRgtLft = true;
    } else if (!attemptedRgtLft && status.leftInvalid) {
      driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
      auto startTime = std::chrono::system_clock::now();
      while (std::chrono::system_clock::now() - startTime < std::chrono::milliseconds(1000)) {
        lidarLogManager.getScanData(nodes, nodeCount);
        status = checkPoint(nodes, nodeCount);
        if (!status.leftInvalid) {
          break;
        }
      }
      attemptedRgtLft = true;
    } else {
      //std::cout << "stopping" << std::endl;
      driveTrain.stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      allClearCnt++;
    }
  } while (allClearCnt < 5);
}

void AutoReturnToStart::rotate() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Rotate";
  double rotationDiff;
  int allClearCnt = 0;
  do {
    double currentRotation = gyroManager.getGyroZ();
    while (currentRotation >= 360) {
      currentRotation -= 360;
    }
    while (currentRotation < 0) {
      currentRotation += 360;
    }
    rotationDiff = currentRotation - m_targetRotation;
    while (rotationDiff > 180) {
      rotationDiff -= 360;
    }
    while (rotationDiff < -180) {
      rotationDiff += 360;
    }

    if (rotationDiff > 5) {
      if (rotationDiff > 30) {
        driveTrain.drive(DriveTrain::RotateRight, DriveTrain::Fast);
      } else {
        driveTrain.drive(DriveTrain::RotateRight, DriveTrain::Slow);
      }
    } else if (rotationDiff < -5) {
      if (rotationDiff < -30) {
        driveTrain.drive(DriveTrain::RotateLeft, DriveTrain::Fast);
      } else {
        driveTrain.drive(DriveTrain::RotateLeft, DriveTrain::Slow);
      }
    } else {
      driveTrain.stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      allClearCnt++;
    }
  } while (allClearCnt < 5);
}

void AutoReturnToStart::setTargetRotation(double targetRotation) {
  while (targetRotation >= 360) {
    targetRotation -= 360;
  }
  while (targetRotation < 0) {
    targetRotation += 360;
  }
  m_targetRotation = targetRotation;
}

AutoReturnToStart::LidarDistances AutoReturnToStart::getDistances(rplidar_response_measurement_node_hq_t *nodes,
                                                                  size_t nodeCount) {
  LidarDistances distances{};
  float frontAverage = 0;
  int frontCnt = 0;
  float backAverage = 0;
  int backCnt = 0;
  float rightAverage = 0;
  int rightCnt = 0;
  float leftAverage = 0;
  int leftCnt = 0;
  for (int i = 0; i < nodeCount; i++) {
    float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
    float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
    if (((angle_in_degrees >= 0 && angle_in_degrees <= 10) ||
        (angle_in_degrees < 360 && angle_in_degrees >= 350)) && nodes[i].quality > 0) {
      backAverage += distance_in_meters;
      backCnt++;
    } else if (angle_in_degrees >= 170 && angle_in_degrees <= 190 && nodes[i].quality > 0) {
      frontAverage += distance_in_meters;
      frontCnt++;
    } else if (angle_in_degrees >= 260 && angle_in_degrees <= 280 && nodes[i].quality > 0) {
      rightAverage += distance_in_meters;
      rightCnt++;
    } else if (angle_in_degrees >= 80 && angle_in_degrees <= 100 && nodes[i].quality > 0) {
      leftAverage += distance_in_meters;
      leftCnt++;
    }
  }
  if (frontCnt > 0) {
    frontAverage /= (float) frontCnt;
  } else {
    frontAverage = std::numeric_limits<float>::max();;
  }
  if (backCnt > 0) {
    backAverage /= (float) backCnt;
  } else {
    backAverage = std::numeric_limits<float>::max();;
  }
  if (rightCnt > 0) {
    rightAverage /= (float) rightCnt;
  } else {
    rightAverage = std::numeric_limits<float>::max();;
  }
  if (leftCnt > 0) {
    leftAverage /= (float) leftCnt;
  } else {
    leftAverage = std::numeric_limits<float>::max();;
  }

  distances.front = frontAverage;
  distances.back = backAverage;
  distances.right = rightAverage;
  distances.left = leftAverage;
  return distances;
}

void AutoReturnToStart::driveToFrdBwdMiddle() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Drive to frd bwd middle";
  float goalCm = 147.f/2/2 + 15;
  float min = (goalCm - 2)/100;
  float max = (goalCm + 2)/100;

  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);

  int allClearCnt = 0;

  do {
    lidarLogManager.getScanData(nodes, nodeCount);
    LidarDistances distances = getDistances(nodes, nodeCount);
    if (distances.front < distances.back) {
      if (distances.front < min) {
        if (min - distances.front > 0.07) {
          driveTrain.drive(DriveTrain::Backward, DriveTrain::Medium);
        } else {
          driveTrain.drive(DriveTrain::Backward, DriveTrain::Slow);
        }
      } else if (distances.front > max) {
        if (distances.front - max > 0.07) {
          driveTrain.drive(DriveTrain::Forward, DriveTrain::Medium);
        } else {
          driveTrain.drive(DriveTrain::Forward, DriveTrain::Slow);
        }
      } else {
        driveTrain.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        allClearCnt++;
      }
    } else {
      if (distances.back < min) {
        if (min - distances.back > 0.07) {
          driveTrain.drive(DriveTrain::Forward, DriveTrain::Medium);
        } else {
          driveTrain.drive(DriveTrain::Forward, DriveTrain::Slow);
        }
      } else if (distances.back > max) {
        if (distances.back - max > 0.07) {
          driveTrain.drive(DriveTrain::Backward, DriveTrain::Medium);
        } else {
          driveTrain.drive(DriveTrain::Backward, DriveTrain::Slow);
        }
      } else {
        driveTrain.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        allClearCnt++;
      }
    }
  } while (allClearCnt < 5);
}

void AutoReturnToStart::ensureRgtLftCorrectDistance() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Ensure rgt lft correct distance";
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);

  int allClearCnt = 0;

  do {
    lidarLogManager.getScanData(nodes, nodeCount);
    LidarDistances distances = getDistances(nodes, nodeCount);
    if (distances.right - distances.left < 0.2 && distances.right - distances.left >= 0) {
      driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
    } else if (distances.left - distances.right < 0.2 && distances.left - distances.right >= 0) {
      driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
    } else if (distances.right < 0.4) {
      driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
    } else if (distances.left < 0.4) {
      driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
    } else {
      driveTrain.stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      allClearCnt++;
    }
  } while (allClearCnt < 5);
}

int countSetBits(int n) {
  int count = 0;

  while (n > 0) {
    // Use bitwise AND to check the least significant bit
    // If it's 1, increment the count
    count += n & 1;

    // Right shift the number to check the next bit
    n >>= 1;
  }

  return count;
}

bool AutoReturnToStart::strideToPath() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Stride to path";
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);

  int allClearCnt = 0;
  bool fail = false;
  auto startTime = std::chrono::system_clock::now();

  do {
    lidarLogManager.getScanData(nodes, nodeCount);
    LidarDistances distances = getDistances(nodes, nodeCount);
    char data = reflectanceSensorManager.getSensorValues() & 0x7E;
    if (std::chrono::system_clock::now() - startTime > std::chrono::seconds(30)) {
      driveTrain.stop();
      fail = true;
    } else if (distances.right - distances.left >= 0 && data == 0) {
      driveTrain.drive(DriveTrain::StrideLeft, DriveTrain::Slow);
    } else if (data == 0) {
      driveTrain.drive(DriveTrain::StrideRight, DriveTrain::Slow);
    } else if (countSetBits(data) <= 3) {
      driveTrain.stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      allClearCnt++;
    } else {
      driveTrain.stop();
      fail = true;
    }
  } while (allClearCnt < 5 && !fail);
  return !fail;
}

int findClosestIndex(const std::array<float, 4>& arr, float target) {
  int closestIndex = 0;  // Initialize the index to the first element
  float minDifference = std::abs(arr[0] - target);  // Initialize the minimum difference

  for (int i = 1; i <4; i++) {
    float currentDifference = std::abs(arr[i] - target);

    // Update the index and minimum difference if a closer value is found
    if (currentDifference < minDifference) {
      closestIndex = i;
      minDifference = currentDifference;
    }
  }

  return closestIndex;
}

bool AutoReturnToStart::goToStart() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Go to start";
  std::vector<State::IntersectionDirection> path;
  std::array<float, 4> distanceRatios = {0.162, 0.455, 1.934, 5.63};
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);
  lidarLogManager.getScanData(nodes, nodeCount);
  LidarDistances distances = getDistances(nodes, nodeCount);
  float distanceRatio = distances.right / distances.left;
  int pos = findClosestIndex(distanceRatios, distanceRatio);
  if (pos == 0) {
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
  } else if (pos == 1) {
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
  } else if (pos == 2) {
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Left);
  } else if (pos == 3) {
    path.push_back(State::IntersectionDirection::Straight);
    path.push_back(State::IntersectionDirection::Right);
    path.push_back(State::IntersectionDirection::Right);
    path.push_back(State::IntersectionDirection::Left);
    path.push_back(State::IntersectionDirection::Left);
  }
  StateManager::getIntersectionState().setPath(path, 1);
  std::chrono::time_point<std::chrono::system_clock> lastIntersectionTime = std::chrono::system_clock::now();
  State *currentState = &StateManager::getStartState();
  currentState->runMotors(driveTrain);
  while (!currentState->isEnd()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    char data = reflectanceSensorManager.getSensorValues() & 0x7E;
    currentState = &currentState->getNextState(data);

    auto timeNow = std::chrono::system_clock::now();

    if (currentState == &StateManager::getIntersectionState()) {
      lastIntersectionTime = timeNow;
    }

    if (timeNow - lastIntersectionTime > std::chrono::seconds (20) &&
        currentState != &StateManager::getEndState() &&
        currentState != &StateManager::getErrorState()) {
      std::cout << "Robot stuck detected during auto return to start" << std::endl;
      BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeNow) <<
                          "Robot stuck detected during auto return to start";
      driveTrain.stop();
      return false;
    }

    currentState->runMotors(driveTrain);
  }
  return currentState->isEndNormal();
}

bool AutoReturnToStart::verifyStart() {
  long long timeLog = std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now() - TimeManager::getStartTime()).count();
  BOOST_LOG(m_logger) << logging::add_value("DataTimeStamp", timeLog) <<
                      "Verify start";
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);
  lidarLogManager.getScanData(nodes, nodeCount);
  LidarDistances distances = getDistances(nodes, nodeCount);
  double rotation = gyroManager.getGyroZ();
  while (rotation >= 360) {
    rotation -= 360;
  }
  while (rotation < 0) {
      rotation += 360;
  }
  float frontGoalCm = 105;
  float frontMin = (frontGoalCm - 9.5f)/100;
  float frontMax = (frontGoalCm + 9.5f)/100;
  float backGoalCm = 42;
  float backMin = (backGoalCm - 8)/100;
  float backMax = (backGoalCm + 8)/100;
  float rightGoalCm = 86;
  float rightMin = (rightGoalCm - 11.5f)/100;
  float rightMax = (rightGoalCm + 11.5f)/100;
  float leftGoalCm = 94;
  float leftMin = (leftGoalCm - 11.5f)/100;
  float leftMax = (leftGoalCm + 11.5f)/100;
  double angleDiff = std::abs(rotation - m_targetRotation);
  while (angleDiff > 180) {
      angleDiff = 360 - angleDiff;
  }
  double rotationTolerance = 18;

  return distances.front >= frontMin &&
         distances.front <= frontMax &&
         distances.back >= backMin &&
         distances.back <= backMax &&
         distances.right >= rightMin &&
         distances.right <= rightMax &&
         distances.left >= leftMin &&
         distances.left <= leftMax;
}

}