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

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace moveit_topp
{

static const std::string ROBOT_DESCRIPTION = "robot_description";

class ToppMoveItDemo
{
public:

  /**
   * \brief Constructor
   */
  ToppMoveItDemo()
    : name_("topp_moveit_demo")
  {
    std::string joint_model_group;

    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    int error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "joint_model_group", joint_model_group);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Load the loader
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

    // Load the robot model
    robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

    // Choose planning group
    jmg_ = robot_model_->getJointModelGroup(joint_model_group);


    ROS_INFO_STREAM_NAMED(name_,"ToppMoveItDemo Ready.");
  }

  void run()
  {
    // Initialize interpolater
    moveit_topp::SplineFitting spline_fitting;

    // Initialize optimizer
    std::vector<double> vel_limits = {0.7895, 0.7895, 0.973, 1.2162, 1.7143, 2.6471, 3.3962};
    std::vector<double> acc_limits = {0.7895, 0.7895, 0.973, 1.2162, 1.7143, 2.6471, 3.3962};
    moveit_topp::MoveItTopp optimizer(vel_limits, acc_limits);


    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr robot_traj(new robot_trajectory::RobotTrajectory(robot_model_, jmg_));
    //double dummy_dt = 1;  // dummy value until parameterization
    //robot_traj->addSuffixWayPoint(interpolated_state, dummy_dt);

    // Compare to MoveIt's default parameterizer
    trajectory_processing::IterativeParabolicTimeParameterization default_time_parameterizer;
    default_time_parameterizer.computeTimeStamps(*robot_traj);


    // Load pre-generated (from Matlab) piecewise polynomial from file
std::size_t num_joints = 7;
spline_fitting.readJointTrajFromFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/moveit_joint_traj.csv", num_joints);

    // Benchmark runtime
    ros::Time start_time = ros::Time::now();

    // Fit a spline to waypoinst
    spline_fitting.fitSpline();

    TOPP::Trajectory orig_trajectory;
    spline_fitting.getPPTrajectory(orig_trajectory);

    bool debug = false;
    if (debug)
    {
      // Write PP trajectory to CSV file (for use with Matlab)
      ROS_WARN_STREAM_NAMED("main", "converting orig trajectory to joint space");
      optimizer.writeTrajectoryToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv",
                                      orig_trajectory);
      return;
    }

    // Time-Optimize with respect to constraints
    TOPP::Trajectory new_trajectory;
    optimizer.optimizeTrajectory(orig_trajectory, new_trajectory);

    // Benchmark runtime
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED("main", "Total time: " << duration << " seconds (" << 1.0/duration << " hz)");

    // Write joint trajectory to CSV file (for use with Matlab)
    optimizer.writeTrajectoryToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv",
                                    new_trajectory);
  }

private:

  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;

  // Desired planning group to work with
  moveit::core::JointModelGroup *jmg_;
}; // end class

} // namespace moveit_topp

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "demo_traj_msg_to_optimal");
  ROS_INFO_STREAM_NAMED("main", "Starting MoveItTopp demo...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit_topp::ToppMoveItDemo demo;
  demo.run();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Finished.");
  ros::shutdown();

  return 0;
}
