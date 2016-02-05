# MoveIt! Time Optimal Path Parameterization

Description: Time optimizes a kinematic path, say from sampling-based motion planning or IK solvers, to be as fast as possible under velocity and acceleration constraints.

Features:

 - Converts a kinematic path into piecewise polynomials using spline fitting
 - Spline fitting accomplished using cubic Hermite interpolant by the algorithm CALCCF from Conte and deBoor.

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder, using the TOPP library by Quang-Cuong Pham

Status:

 * [![Build Status](https://travis-ci.org/davetcoleman/moveit_topp.svg)](https://travis-ci.org/davetcoleman/moveit_topp) Indigo Travis CI
 * [![Devel Job Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-moveit_topp)](http://jenkins.ros.org/job/devel-indigo-moveit_topp) Indigo Devel Job Status
 * [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-moveit-topp_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-moveit-topp_binarydeb_trusty_amd64/) Indigo AMD64 Debian Job Status

![](resources/screenshot.png)

## Licensing

Be aware there are multiple licenses in this project:

Spline Fitting - GNU LGPL v3
TOPP Library - GNU LGPL v3
ROS Package - BSD 3-Clause

## Install

### Ubuntu Debian

Not available yet:

    sudo apt-get install ros-indigo-moveit-topp

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro indigo

## Code API

See [Class Reference](http://docs.ros.org/indigo/api/moveit_topp/html/)

## Usage

### Demo: Convert Piecewise Polynomial Trajectory to Time-Optimal Discretized Path

- Use the Matlab script ``scripts/convert_joint_traj_to_pp.m`` to generate ``data/matlab_pp_traj.csv``
- Convert the file into discretized optimized waypoints in C++ using TOPP via:

        rosrun moveit_topp demo_pp_to_optimal

- This generates ``data/topp_optimized_traj.csv``
- View the generated discretized waypoints:

        matlab scripts/display_TOPP_traj.m

### Demo: Convert Joint Trajectory Waypoints into Piecewise Polynomial Trajectory

- Generate a joint-space waypoints trajectory using MoveIt! and save into ``data/matlab_joint_traj.csv``
- Convert the file into piecewise polynomials in C++ using via:

        rosrun moveit_topp demo_joint_traj_to_pp

- This generates ``data/spline_pp_traj.csv``
- View the generated PP spline in Matlab:

        matlab scripts/display_PP.m

### Demo: Convert Joint Trajectory Waypoints into Time-Optimal Discretized Path

- Generate a joint-space waypoints trajectory using MoveIt! and save into ``data/matlab_joint_traj.csv``
- Convert the file into discretized optimized waypoints in C++ using TOPP via:

        rosrun moveit_topp demo_joint_traj_to_optimal

- This generates ``data/topp_optimized_traj.csv``
- View the generated discretized waypoints:

        matlab scripts/display_TOPP_traj.m

## Demo: Convert ROS Trajectory Message Waypoints into Time-Optimal Discretized Path

UNFINISHED

- Generate a joint-space waypoints trajectory using MoveIt! and save into ``data/matlab_joint_traj.csv``
- Convert the file into discretized optimized waypoints in C++ using TOPP via:

        rosrun moveit_topp demo_traj_msg_to_optimal

- This generates ``data/topp_optimized_traj.csv``
- View the generated discretized waypoints:

        matlab scripts/display_TOPP_traj.m

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
