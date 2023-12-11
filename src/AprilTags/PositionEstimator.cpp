#include "AprilTags/PositionEstimator.hpp"

#include <apriltag_pose.h>
#include <opencv2/opencv.hpp>
#include <numbers>
#include <stdexcept>
#include <iostream>

using namespace cv;

namespace RobotCode::AprilTags {

double truncateAngle(double angle) {
  while (angle > std::numbers::pi) {
    angle -= 2*std::numbers::pi;
  }
  while (angle < -std::numbers::pi) {
    angle += 2*std::numbers::pi;
  }
  return angle;
}

PositionEstimator::RobotPosition PositionEstimator::getRobotPosition(std::vector<std::pair<CameraProperties, zarray_t*>> detections) {
  apriltag_pose_t minCostDetectionPose;
  int minCostDetectionId = -1;
  double minCost = std::numeric_limits<double>::max();
  double minErr;
  CameraProperties minDistanceDetectionCameraProperties{};
  for (auto detection : detections) {
    for (int i = 0; i < zarray_size(detection.second); i++) {
      apriltag_detection_t *det;
      zarray_get(detection.second, i, &det);

      if (!tagProperties.count(det->id)) {
        throw std::runtime_error("AprilTag id not found in tagProperties.");
      }

      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = tagProperties[det->id].size;
      info.fx = detection.first.fx;
      info.fy = detection.first.fy;
      info.cx = detection.first.cx;
      info.cy = detection.first.cy;

      apriltag_pose_t pose;
      double err = estimate_tag_pose(&info, &pose);
      double cost = std::abs(std::atan(pose.t->data[0]));
      /*double cost = std::abs(-std::atan2(-pose.R->data[6],
                                    std::sqrt(
                                        pose.R->data[7]*pose.R->data[7] +
                                            pose.R->data[8]*pose.R->data[8]
                                    )
      ));*/
          //std::sqrt(pose.t->data[0] * pose.t->data[0] + pose.t->data[1] * pose.t->data[1] + pose.t->data[2] * pose.t->data[2]);

      if (cost < minCost) {
        minCost = cost;
        minCostDetectionId = det->id;
        minCostDetectionPose = pose;
        minDistanceDetectionCameraProperties = detection.first;
        minErr = err;
      }

      //std::cout << "Pose: " << pose.t->data[0] << ", " << pose.t->data[1] << ", " << pose.t->data[2] << std::endl;
    }
  }
  //std::cout << "ID" << minCostDetectionId << std::endl;
  if (minCostDetectionId == -1) {
    throw std::runtime_error("No AprilTag detections found.");
  }

  // apriltag properties
  Mat aprilTagRotationVector(1, 3, CV_64F);
  aprilTagRotationVector.at<double>(0, 0) = tagProperties[minCostDetectionId].roll;
  aprilTagRotationVector.at<double>(0, 1) = tagProperties[minCostDetectionId].pitch;
  aprilTagRotationVector.at<double>(0, 2) = tagProperties[minCostDetectionId].yaw;

  Mat aprilTagRotationMatrix(3, 3, CV_64F);
  Rodrigues(aprilTagRotationVector, aprilTagRotationMatrix);

  Mat aprilTagPose(3, 1, CV_64F);
  aprilTagPose.at<double>(0, 0) = tagProperties[minCostDetectionId].x;
  aprilTagPose.at<double>(1, 0) = tagProperties[minCostDetectionId].y;
  aprilTagPose.at<double>(2, 0) = tagProperties[minCostDetectionId].z;

  // camera detection
  Mat cameraDetectedRotationVector(1, 3, CV_64F);
  cameraDetectedRotationVector.at<double>(0, 0) = 0;//-std::atan2(minDistanceDetectionPose.R->data[3], minDistanceDetectionPose.R->data[0]);
  cameraDetectedRotationVector.at<double>(0, 1) = 0;//std::atan2(minDistanceDetectionPose.R->data[7], minDistanceDetectionPose.R->data[8]);
  cameraDetectedRotationVector.at<double>(0, 2) = -std::atan2(-minCostDetectionPose.R->data[6],
                                                              std::sqrt(
                                                                  minCostDetectionPose.R->data[7]*minCostDetectionPose.R->data[7] +
                                                                  minCostDetectionPose.R->data[8]*minCostDetectionPose.R->data[8]
                                                                  )
                                                              );

  Mat cameraDetectedRotationMatrix(3, 3, CV_64F);
  Rodrigues(cameraDetectedRotationVector, cameraDetectedRotationMatrix);

  Mat cameraDetectedPose(3, 1, CV_64F);
  cameraDetectedPose.at<double>(0, 0) = minCostDetectionPose.t->data[2];
  cameraDetectedPose.at<double>(1, 0) = -minCostDetectionPose.t->data[0];
  cameraDetectedPose.at<double>(2, 0) = 0;//minDistanceDetectionPose.t->data[1];

  //std::cout << "Camera detected pose: " << cameraDetectedPose.at<double>(0, 0) << ", " << cameraDetectedPose.at<double>(1, 0) << ", " << cameraDetectedPose.at<double>(2, 0) << std::endl;
  //std::cout << "Camera rotation: " << cameraDetectedRotationVector.at<double>(0, 0)*180/3.14 << ", " << cameraDetectedRotationVector.at<double>(0, 1)*180/3.14 << ", " << cameraDetectedRotationVector.at<double>(0, 2)*180/3.14 << std::endl;

  // calculate camera pose relative to tag
  Mat cameraPoseRelativeToTag = cameraDetectedRotationMatrix * cameraDetectedPose;
  cameraPoseRelativeToTag.at<double>(0, 0) = -cameraPoseRelativeToTag.at<double>(0, 0);
  //cameraPoseRelativeToTag.at<double>(1, 0) = -cameraPoseRelativeToTag.at<double>(1, 0);
  Mat cameraRotationRelativeToTag = -cameraDetectedRotationVector;

  //std::cout << "Camera pose relative to tag" << cameraPoseRelativeToTag.at<double>(0, 0)
  //    << ", " << cameraPoseRelativeToTag.at<double>(1, 0)
  //      << ", " << cameraPoseRelativeToTag.at<double>(2, 0) << std::endl;

  // calculate camera pose
  Mat cameraPose = aprilTagRotationMatrix * cameraPoseRelativeToTag;// + aprilTagPose;
  //std::cout << "Camera pose: " << cameraPose.at<double>(0, 0) << ", " << cameraPose.at<double>(1, 0) << ", " << cameraPose.at<double>(2, 0) << std::endl;
  cameraPose += aprilTagPose;
  Mat cameraRotationVector = aprilTagRotationVector + cameraRotationRelativeToTag;

  //std::cout << "Camera pose: " << cameraPose.at<double>(0, 0) << ", " << cameraPose.at<double>(1, 0) << ", " << cameraPose.at<double>(2, 0) << std::endl;
  //std::cout << "Err: " << minErr << std::endl;

  Mat cameraRotationMatrix(3, 3, CV_64F);
  Rodrigues(cameraRotationVector, cameraRotationMatrix);

  // robot pose relative to camera
  Mat robotRotationVector(1, 3, CV_64F);
  robotRotationVector.at<double>(0, 0) = -minDistanceDetectionCameraProperties.roll;
  robotRotationVector.at<double>(0, 1) = -minDistanceDetectionCameraProperties.pitch;
  robotRotationVector.at<double>(0, 2) = -minDistanceDetectionCameraProperties.yaw;

  Mat robotRotationMatrix(3, 3, CV_64F);
  Rodrigues(robotRotationVector, robotRotationMatrix);

  Mat robotPoseRelativeToCamera(3, 1, CV_64F);
  robotPoseRelativeToCamera.at<double>(0, 0) = -minDistanceDetectionCameraProperties.x;
  robotPoseRelativeToCamera.at<double>(1, 0) = -minDistanceDetectionCameraProperties.y;
  robotPoseRelativeToCamera.at<double>(2, 0) = -minDistanceDetectionCameraProperties.z;

  // calculate robot pose
  Mat robotPose = cameraRotationMatrix * robotPoseRelativeToCamera + cameraPose;
  Mat robotRotation = robotRotationVector + cameraRotationVector;

  RobotPosition robotPoseObject{};
  robotPoseObject.id = minCostDetectionId;
  robotPoseObject.err = minErr;
  robotPoseObject.x = robotPose.at<double>(0, 0);
  robotPoseObject.y = robotPose.at<double>(1, 0);
  robotPoseObject.z = 0;//robotPose.at<double>(2, 0); // not needed
  robotPoseObject.roll = 0;//robotRotation.at<double>(0, 0); // not getting correct value, need to fix, but getting only yaw is enough for now
  robotPoseObject.pitch = 0;//robotRotation.at<double>(0, 1); // not getting correct value, need to fix, but getting only yaw is enough for now
  robotPoseObject.yaw = truncateAngle(robotRotation.at<double>(0, 2));
  robotPoseObject.tagRelativeX = minCostDetectionPose.t->data[2];
  robotPoseObject.tagRelativeY = minCostDetectionPose.t->data[0];
  robotPoseObject.tagRelativeZ = minCostDetectionPose.t->data[1];

  return robotPoseObject;
}

double tagSize = 0.053;

std::map<int, PositionEstimator::AprilTagProperties> PositionEstimator::tagProperties{
    {
        0,
        {
            tagSize,
            0,
            0.35,
            0,
            0.0,
            0.0,
            180*std::numbers::pi/180
        }
    },
    {
        1,
        {
            tagSize,
            0,
            0.795,
            0,
            0.0,
            0.0,
            180*std::numbers::pi/180
        }
    },
    {
      2,
          {
              tagSize,
              0,
              1.35,
              0,
              0.0,
              0.0,
              180*std::numbers::pi/180
          }
    },
    {
        3,
        {
            tagSize,
            0.225,
            1.80,
            0,
            0.0,
            0.0,
            90*std::numbers::pi/180
        }
    },
    {
        4,
        {
            tagSize,
            0.715,
            1.80,
            0,
            0.0,
            0.0,
            90*std::numbers::pi/180
        }
    },
    {
        5,
        {
            tagSize,
            1.18,
            1.78,
            0,
            0.0,
            0.0,
            90*std::numbers::pi/180
        }
    },
    {
        6,
        {
            tagSize,
            1.49,
            1.305,
            0,
            0.0,
            0.0,
            0*std::numbers::pi/180
        }
    },
    {
        7,
        {
            tagSize,
            1.49,
            0.92,
            0,
            0.0,
            0.0,
            0*std::numbers::pi/180
        }
    },
    {
        8,
        {
            tagSize,
            1.49,
            0.415,
            0,
            0.0,
            0.0,
            0*std::numbers::pi/180
        }
    },
    {
        9,
        {
            tagSize,
            1.13,
            0,
            0,
            0.0,
            0.0,
            270*std::numbers::pi/180
        }
    },
    {
        10,
        {
            tagSize,
            0.71,
            0,
            0,
            0.0,
            0.0,
            270*std::numbers::pi/180
        }
    },
    {
        11,
        {
            tagSize,
            0.397,
            0,
            0,
            0.0,
            0.0,
            270*std::numbers::pi/180
        }
    }
};

}