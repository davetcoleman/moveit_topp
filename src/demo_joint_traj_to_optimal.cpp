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
   Desc:   Test for converting a discretized joint trajectory into piecewise polynomials using splines
*/

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// this package
#include <moveit_topp/spline_fitting.h>
#include <moveit_topp/moveit_topp.h>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "demo_joint_traj_to_optimal");
  ROS_INFO_STREAM_NAMED("main", "Starting MoveItTopp demo...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  moveit_topp::SplineFitting spline_fitting;

  // Load pre-generated (from Matlab) piecewise polynomial from file
  spline_fitting.readJointTrajFromFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/moveit_joint_traj.csv");

  // Fit a spline to waypoinst
  spline_fitting.fitSpline();
  return 0;

  TOPP::Trajectory orig_trajectory;
  spline_fitting.getPPTrajectory(orig_trajectory);

  // Initialize optimizer
  moveit_topp::MoveItTopp optimizer;

  bool debug = true;
  if (debug)
  {
    // Write PP trajectory to CSV file (for use with Matlab)
    ROS_WARN_STREAM_NAMED("main", "converting orig trajectory to joint space");
    optimizer.writeTrajectoryToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv",
                                    orig_trajectory);
    return 0;
  }

  // Time-Optimize with respect to constraints
  TOPP::Trajectory new_trajectory;
  optimizer.optimizeTrajectory(orig_trajectory, new_trajectory);

  // Write joint trajectory to CSV file (for use with Matlab)
  //optimizer.writeTrajectoryToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv",
  //new_trajectory);

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Finished.");
  ros::shutdown();

  return 0;
}
