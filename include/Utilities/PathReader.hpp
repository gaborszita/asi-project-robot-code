#ifndef ROBOT_CODE_UTILITIES_PATHREADER_HPP
#define ROBOT_CODE_UTILITIES_PATHREADER_HPP

#include <string>
#include <vector>
#include "RobotControl/LineFollowerFSM.hpp"

namespace RobotCode::Utilities {

class PathReader {
 public:
  struct PathParameters {
    std::string pathName;
    std::vector<RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection> path;
    int pathRep;
  };

  static void readPath(const std::string& filePath,
                       std::string& pathName,
                       std::vector<RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection> &path,
                       int& pathRep);
  static void readPath(const std::string& filePath,
                       PathParameters& pathParameters);
};

}

#endif //ROBOT_CODE_UTILITIES_PATHREADER_HPP
