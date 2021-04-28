# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program  
author : s.aparajith@live.com  
date: 27.04.2021  

build matrix : arm64 and amd64 on linux 20.04  
[![Build Status](https://travis-ci.org/Aparajith-S/Extended-Kalman-Filter-Project.svg?branch=master)](https://travis-ci.org/Aparajith-S/Extended-Kalman-Filter-Project)

---
In this project a kalman filter will be utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Code

`main.h`
the main entry point of the application

`FusionEKF.h` contains the class `FusionEKF` with the following member function
-  `ProcessMeasurement(...)` which runs one step of the EKF fusioning algorithm 
  of predict-update cycle using the fed in measurements.

`kalman-filter.h` contains the class `KalmanFilter` which contains the algorithm to run a  predict and update measurement cycle of an Extended kalman filter.  
- `Predict()` does the prediction  
- `Update(...)` does the update using the Kalman filter update equations  
- `UpdateEKF(...)` does the update for the Extended Kalman Filter  
- `KfCommonUpdates(...)` is a common update function for the equations common to the EKF and KF  

`tools.h` contains the class `Tools` which have methods to compute
- `CalculateRMSE` computes the root mean squared error using the measurements and the ground-truth
- `CalculateJacobian` computes the jacobian matrix for the EKF.

`measurement_package.h` contains the class `MeasurementPackage` that help in fetching and handling raw radar and lidar data. 

`types.h` contain the fundamental datatypes used in this project.

---

## Results

[bothSensors1]: ./Docs/sensor.JPG "fus1"
[bothSensors2]: ./Docs/sensor2.JPG "fus2"
[Radar]: ./Docs/onlyRadar.JPG  "radar"
[Radar2]: ./Docs/onlyradar_2.JPG "radar2"
[Lidar]: ./Docs/onlyLidar.JPG "lidar1"
[Lidar2]: ./Docs/onlyLidar2.JPG "lidar2"


| Sensors  | dataset 1 | dataset 2 | 
|:------:|:---------:|:---------:|
| RADAR + LIDAR |![sensfusi][bothSensors1]| ![sensfusi][bothSensors2]|
|RADAR|![sensfusi][Radar]| ![sensfusi][Radar2]|
|LIDAR|![sensfusi][Lidar]| ![sensfusi][Lidar2]|

## Observation
From the above table of results:

- it was noted from the RMSE readings the LIDAR seems to fare better than the RADAR measurements 
- However, for x component of the velocity seems to fare a bit better in the RADAR case than the LIDAR case 
- With the fusioning algorithm, with both the RADAR and LIDAR measurements the RMSE of both positions and velocities were within the [.11, .11, 0.52, 0.52] margin as expected in the rubrik.