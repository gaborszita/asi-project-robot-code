#ifndef ROBOT_CODE_APRILTAGS_POSITIONESTIMATOR_HPP
#define ROBOT_CODE_APRILTAGS_POSITIONESTIMATOR_HPP

#include <vector>
#include <utility>
#include <apriltag/apriltag.h>
#include <map>

namespace RobotCode::AprilTags {

class PositionEstimator {
 public:
  struct CameraProperties {
    double fx;
    double fy;
    double cx;
    double cy;

    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };
  struct RobotPosition {
    double id;
    double x;
    double y;
    double z;
    double tagRelativeX;
    double tagRelativeY;
    double tagRelativeZ;
    double roll;
    double pitch;
    double yaw;
    double err;
  };
  struct AprilTagProperties {
    double size;
    double x; // apriltag center x
    double y; // apriltag center y
    double z; // apriltag center z
    double roll; // apriltag roll in radians
    double pitch; // apriltag pitch in radians
    double yaw; // apriltag yaw in radians
  };
  static RobotPosition getRobotPosition(std::vector<std::pair<CameraProperties, zarray_t*>> detections);
 private:
  static std::map<int, AprilTagProperties> tagProperties;
};

}

#endif //ROBOT_CODE_APRILTAGS_POSITIONESTIMATOR_HPP
