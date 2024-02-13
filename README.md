# ASI project robot code

This repository is part of my work for my Advanced Science Investigation class at Los Altos High School. This repository contains the code that runs on the robot.

## The robot

The robot has the following hardware:
- Raspberry Pi 4 Model B
- RPLIDAR A1
- Adafruit DC & Stepper Motor HAT for Raspberry Pi
- Battery packs
- Two cameras for AprilTag localization
- MPU6050 gyro sensor
- Logitech mouse as optic sensor for localization

## Required packages

- Boost C++ logging library
- [AprilTag library](https://github.com/AprilRobotics/apriltag?tab=readme-ov-file#install)
- OpenCV for camera capture and AprilTag detection
- CMake for compiling (GNU Make is recommended)

## Compiling

This project uses CMake. It is recommended to use GNU make to build this project:

```
cmake -G "Unix Makefiles" path-to-source
make
```

The executable will be named `robot_code`.