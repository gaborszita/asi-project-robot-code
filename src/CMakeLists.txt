set(CMAKE_CXX_STANDARD 20)

add_executable(
  robot_code
  main.cpp
  DeviceManagers/LidarManager.cpp
  DeviceManagers/GyroManager.cpp
  DeviceManagers/MouseManager.cpp
  DeviceManagers/ReflectanceSensorManager.cpp
  DeviceManagers/DriveMotorManager.cpp
  Utilities/PID.cpp
  Utilities/TimeManager.cpp
  Utilities/PathReader.cpp
  logging/LoggingController.cpp
  logging/LidarLogManager.cpp
  RobotControl/DeviceManager.cpp
  RobotControl/DriveTrain.cpp
  RobotControl/LineFollower.cpp
  RobotControl/LineFollowerFSM.cpp
  RobotControl/AutoReturnToStart.cpp
  RobotControl/LineFollowerFSMAutoReturnToStart.cpp
  AprilTags/PositionEstimator.cpp
  AprilTags/AprilTagManager.cpp
)
target_include_directories(robot_code PRIVATE ../include)

# include FetchContent and ExternalProject
include(FetchContent)
include(ExternalProject)

# Adafruit Motor Hat Library
FetchContent_Declare(
  AdafruitStepperMotorHAT_CPP
  GIT_REPOSITORY https://github.com/gaborszita/AdafruitStepperMotorHAT_CPP.git
  GIT_TAG 1df858e58fbc452ceffdc7d101e050c8131aefd2
)
FetchContent_MakeAvailable(AdafruitStepperMotorHAT_CPP)

target_include_directories(robot_code PRIVATE ${adafruitsteppermotorhat_cpp_SOURCE_DIR})
target_link_libraries(robot_code PRIVATE Adafruit_MotorHAT)

# RPLIDAR SDK
ExternalProject_Add(
  rplidar_sdk_proj
  GIT_REPOSITORY https://github.com/gaborszita/rplidar_sdk
  GIT_TAG 66397cc5b0fb4db91070331be8ef7865c487e77c
  BUILD_COMMAND make -C sdk
  CONFIGURE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_IN_SOURCE TRUE
)

ExternalProject_Get_Property(rplidar_sdk_proj SOURCE_DIR BINARY_DIR)
target_include_directories(robot_code PRIVATE ${SOURCE_DIR}/sdk/sdk/include)
target_link_directories(robot_code PRIVATE ${BINARY_DIR}/sdk/output/${CMAKE_SYSTEM_NAME}/Release)
target_link_libraries(robot_code PRIVATE rplidar_sdk)
target_link_libraries(robot_code PRIVATE pthread) # RPLIDAR SDK needs pthread
add_dependencies(robot_code rplidar_sdk_proj)

find_package(
  Boost
  REQUIRED
  COMPONENTS log
)

target_include_directories(robot_code PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(robot_code PRIVATE Boost::log)

target_link_libraries(robot_code PRIVATE atomic)

find_package(OpenCV REQUIRED)
target_include_directories(robot_code PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(robot_code PRIVATE ${OpenCV_LIBS})

find_package(apriltag REQUIRED)
target_include_directories(robot_code PRIVATE ${apriltag_INCLUDE_DIRS})
target_link_libraries(robot_code PRIVATE ${apriltag_LIBS})
target_link_libraries(robot_code PRIVATE "apriltag")
message(STATUS ${apriltag_LIBS})