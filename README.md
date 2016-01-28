# MoveIt! Time Optimal Path Parameterization

Description: TODO

Features:

 - TODO

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder, using the TOPP library by Quang-Cuong Pham

Status:

 * [![Build Status](https://travis-ci.org/davetcoleman/moveit_topp.svg)](https://travis-ci.org/davetcoleman/moveit_topp) Indigo Travis CI
 * [![Devel Job Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-moveit_topp)](http://jenkins.ros.org/job/devel-indigo-moveit_topp) Indigo Devel Job Status
 * [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-moveit-topp_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-moveit-topp_binarydeb_trusty_amd64/) Indigo AMD64 Debian Job Status

![](resources/screenshot.png)

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

TODO

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
