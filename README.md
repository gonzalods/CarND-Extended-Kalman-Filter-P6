# CarND-Extended-Kalman-Filter-P6
Extended Kalman Filter Project of Nanodegree Self-Driving Car Engineer

# **Proyect: Extended Kalman Filters**

---

## **Overview**

This is the sixth proyect of Nanodegree Self-Driving Car Engineer.

The objective of the project is to detect the position and speed of an object that is detected by a radar and lidar sensors.
The sensor data comes from a simulator provided for Udacity that is connected by WebSocket, or from a file in the folder ``data``.

To do this an Extended Kalman Filter is implemented.

---

## **Software**
- *C++* as programming language.
- *[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)* as C++ template library for linear algebra.
- *[uWebSocketIO](https://github.com/uWebSockets/uWebSockets)* as a WebSocket and HTTP implementation for clients and servers.
- *[CMake](https://cmake.org/)* as tool to build package software.

## **File List**

The project consists of the following files:

- *src*: folder with the source files that implement the Extended Kalman Filter.
  * *src/main.cpp* main for simulation version.
  * *src/file.cpp* main for file version.
- *src/Eigen*: folder with the source files of the Eigen library. 
- *data*: folder with the file containing the sensor data.
- *CMakeList.txt*: CMake's configuration file for simulation version of data sensors.
- *CMakeList_file.txt*: CMake's configuration file for file version of data sensors.

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: 
   * for simulation version: `cmake .. && make` 
   * for file version:
     * rename CMakeList.txt (for example CMakeListSim.txt).
     * rename CMakeList_file.txt to CMakeList.txt.
     * `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `