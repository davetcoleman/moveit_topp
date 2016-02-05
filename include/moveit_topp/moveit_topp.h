/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Wrapper for time opitmal path parameterization
*/

#ifndef MOVEIT_TOPP_MOVEIT_TOPP_H
#define MOVEIT_TOPP_MOVEIT_TOPP_H

// C++
#include <string>
#include <fstream>
#include <streambuf>

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// TOPP
#include <TOPP/src/TOPP.h>
#include <TOPP/src/KinematicLimits.h>
#include <TOPP/src/TorqueLimits.h>
#include <TOPP/src/PolygonConstraints.h>

// Spline
#include <spline/spline.hpp>

// MoveIt
#include <moveit/robot_trajectory/robot_trajectory.h>

// this package
#include <moveit_topp/spline_fitting.h>

namespace moveit_topp
{

class MoveItTopp
{
public:

  /**
   * \brief Constructor
   */
  MoveItTopp(const moveit::core::JointModelGroup* jmg);

  /**
   * \brief Constructor
   */
  MoveItTopp(const std::vector<double> &vel_limits, const std::vector<double> &acc_limits);

  void init(const std::vector<double> &vel_limits, const std::vector<double> &acc_limits);

  void computeTimeStamps(robot_trajectory::RobotTrajectory& robot_traj);

  void convertMoveItTrajToPP(const robot_trajectory::RobotTrajectory& robot_traj);

  void readPPTrajFromFile(const std::string& filename, TOPP::Trajectory &trajectory);

  void optimizeTrajectory(const TOPP::Trajectory &old_trajectory, TOPP::Trajectory &new_trajectory);

  bool writeTrajectoryToFile(const std::string &file_path, const TOPP::Trajectory &trajectory);

  bool convertTrajToMoveItTraj(robot_trajectory::RobotTrajectory& robot_traj, const TOPP::Trajectory &trajectory);

private:

  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "moveit_topp";

  // A shared node handle
  ros::NodeHandle nh_;

  // TOPP vars
  boost::shared_ptr<TOPP::Constraints> pconstraints_;

  // Group of joints to use
  const moveit::core::JointModelGroup* jmg_;

  // Initialize interpolater
  moveit_topp::SplineFitting spline_fitting_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<MoveItTopp> MoveItToppPtr;
typedef boost::shared_ptr<const MoveItTopp> MoveItToppConstPtr;

} // namespace moveit_topp

#endif  // MOVEIT_TOPP_MOVEIT_TOPP_H
