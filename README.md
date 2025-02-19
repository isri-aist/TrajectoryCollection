**This is the branch for ROS1.**

# [TrajectoryCollection](https://github.com/isri-aist/TrajectoryCollection)
Trajectory functions for robot control

[![CI-standalone](https://github.com/isri-aist/TrajectoryCollection/actions/workflows/ci-standalone.yaml/badge.svg)](https://github.com/isri-aist/TrajectoryCollection/actions/workflows/ci-standalone.yaml)
[![CI-catkin](https://github.com/isri-aist/TrajectoryCollection/actions/workflows/ci-catkin.yaml/badge.svg)](https://github.com/isri-aist/TrajectoryCollection/actions/workflows/ci-catkin.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/TrajectoryCollection/)

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)

## Trajectory list
- General functions
  - [Polynomial function](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1Polynomial.html#details)
  - [Cubic spline](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1CubicSpline.html#details)
  - [Cubic Hermite spline](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1CubicHermiteSpline.html#details)
    - Supports [monotone cubic interpolation](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1CubicHermiteSpline.html#a16e338fa43dfa5abc78b3103a7bf28db)
  - [Piecewise function](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1PiecewiseFunc.html#details) consisting of the above functions
- Point-to-point interpolations
  - [Cubic interpolation](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1CubicInterpolator.html#details)
  - [Bang-bang control based interpolation (trapezoidal velocity profile)](https://isri-aist.github.io/TrajectoryCollection/doxygen/classTrajColl_1_1BangBangInterpolator.html#details)
