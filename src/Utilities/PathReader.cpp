#include "Utilities/PathReader.hpp"

#include <fstream>

namespace RobotCode::Utilities {

void PathReader::readPath(const std::string& filePath,
                          std::string& pathName,
                          std::vector<RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection> &path,
                          int& pathRep) {
  // filePath is a CSV file with the following format:
  // <path name>,<number of times to repeat path>,<direction>,<direction>,<direction>,...
  // direction is either "Right", "Straight", or "Left"

  std::fstream file;
  file.open(filePath, std::ios::in);

  std::string line;
  std::getline(file, line);
  std::string delimiter = ",";
  size_t pos;
  std::string token;
  int i = 0;
  while ((pos = line.find(delimiter)) != std::string::npos) {
    token = line.substr(0, pos);
    if (i == 0) {
      pathName = token;
    } else if (i == 1) {
      pathRep = std::stoi(token);
    } else {
      processIntersection(token, path);
    }
    line.erase(0, pos + delimiter.length());
    i++;
  }
  processIntersection(line, path);
  file.close();
}

void PathReader::readPath(const std::string& filePath,
                          PathParameters& pathParameters) {
  readPath(filePath, pathParameters.pathName, pathParameters.path, pathParameters.pathRep);
}

void PathReader::processIntersection(const std::string &token,
                                     std::vector<RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection> &path) {
  if (token == "Right") {
    path.push_back(RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection::Right);
  } else if (token == "Straight") {
    path.push_back(RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection::Straight);
  } else if (token == "Left") {
    path.push_back(RobotCode::RobotControl::LineFollowerFSM::State::IntersectionDirection::Left);
  } else {
    throw std::runtime_error("Error reading path file: Invalid path direction: " + token);
  }
}

}