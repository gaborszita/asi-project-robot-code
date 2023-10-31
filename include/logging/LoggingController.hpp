#ifndef ROBOT_CODE_LOGGING_LOGGINGCONTROLLER_HPP
#define ROBOT_CODE_LOGGING_LOGGINGCONTROLLER_HPP

#include <string>

namespace RobotCode::Logging {

class LoggingController {
 public:
  static void init(const std::string& logFileName);
};

}

#endif //ROBOT_CODE_LOGGING_LOGGINGCONTROLLER_HPP