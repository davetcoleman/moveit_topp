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

// this package
#include <moveit_topp/moveit_topp.h>

namespace moveit_topp
{

MoveItTopp::MoveItTopp(ros::NodeHandle &nh, const moveit::core::JointModelGroup* jmg)
  : nh_(nh)
  , jmg_(jmg)
{
  std::vector<double> vel_limits;
  std::vector<double> acc_limits;
  for (std::size_t i = 0; i < jmg->getActiveJointModels().size(); ++i)
  {
    const moveit::core::JointModel::Bounds& bounds = jmg->getActiveJointModels()[i]->getVariableBounds();
    if (bounds.size() != 1)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid number of bounds");
      continue;
    }
    const moveit::core::VariableBounds& bound = bounds.front();
    vel_limits.push_back(bound.max_velocity_);
    acc_limits.push_back(bound.max_acceleration_);
    //std::cout << "bound.max_velocity_: " << bound.max_velocity_ << std::endl;
    //std::cout << "bound.max_acceleration_: " << bound.max_acceleration_ << std::endl;
  }

  // Resize datastructures for convertMoveItTrajToPP()
  ROS_INFO_STREAM_NAMED(name_, "Joint model group var count: " << jmg_->getVariableCount());
  tmp_joint_positions_.resize(jmg_->getVariableCount());
  tmp_jmg_positions_.resize(jmg_->getVariableCount());

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/write_coeff_to_file", debug_write_coeff_to_file_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/write_joint_traj_to_file", debug_write_joint_traj_to_file_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  init(vel_limits, acc_limits);
}

MoveItTopp::MoveItTopp(const std::vector<double> &vel_limits, const std::vector<double> &acc_limits)
{
  init(vel_limits, acc_limits);
}

void MoveItTopp::init(const std::vector<double> &vel_limits, const std::vector<double> &acc_limits)
{
  // Error checking
  if (vel_limits.size() != acc_limits.size() ||
      vel_limits.empty())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Invalid vel/acc limits");
    exit(-1);
  }

  // Convert constraints into a weird string format required by TOPP
  double discrtimestep = 0.005; // TODO(davetcoleman): no hardcode
  std::string constraint_string = std::to_string(discrtimestep);
  constraint_string.append("\n");
  for (std::size_t i = 0; i < vel_limits.size(); ++i)
  {
    std::string temp = std::to_string(vel_limits[i]).append(" ");
    constraint_string.append(temp); // velocity
  }
  constraint_string.append("\n");
  for (std::size_t i = 0; i < vel_limits.size(); ++i)
  {
    std::string temp = std::to_string(acc_limits[i]).append(" ");
    constraint_string.append(temp); // acceleration
  }
  bool verbose = false;
  if (verbose)
  {
    std::cout << "constraint_string:\n" << constraint_string << std::endl;
  }

  // Setup constraints
  pconstraints_.reset(new TOPP::KinematicLimits(constraint_string));

  ROS_INFO_STREAM_NAMED(name_,"MoveItTopp Ready.");
}

void MoveItTopp::computeTimeStamps(robot_trajectory::RobotTrajectory& robot_traj)
{
  // Convert MoveIt format to Spline format and send to spline fitting
  convertMoveItTrajToPP(robot_traj);

  // Fit a spline to waypoinst
  spline_fitting_.fitSpline();

  // Write coefficients to file (for use with Matlab)
  if (debug_write_coeff_to_file_)
  {
    spline_fitting_.writeCoefficientsToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/spline_pp_traj.csv");
  }

  // Convert to TOPP format
  TOPP::Trajectory orig_trajectory;
  spline_fitting_.getPPTrajectory(orig_trajectory);

  // Time-Optimize with respect to constraints
  TOPP::Trajectory new_trajectory;
  optimizeTrajectory(orig_trajectory, new_trajectory);

  // Debug
  if (debug_write_joint_traj_to_file_)
  {
    writeTrajectoryToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv",
                          new_trajectory);
  }

  // Convert trajectory back to MoveIt's format
  convertTrajToMoveItTraj(robot_traj, new_trajectory);
}

void MoveItTopp::convertMoveItTrajToPP(const robot_trajectory::RobotTrajectory& robot_traj)
{
  // Populate joint_positions with robot trajectory

  // Resize
  ROS_INFO_STREAM_NAMED(name_, "Converting MoveIt! robot trajectory with " << jmg_->getVariableCount() << " dof");
  tmp_timestamps_.resize(robot_traj.getWayPointCount());
  for (std::size_t joint_id = 0; joint_id < jmg_->getVariableCount(); ++joint_id) // for each joint
  {
    tmp_joint_positions_[joint_id].resize(robot_traj.getWayPointCount());
  }

  // Copy - this requires transposing the matrix
  for (std::size_t i = 0; i < robot_traj.getWayPointCount(); ++i) // for each waypoint
  {
    robot_traj.getWayPoint(i).copyJointGroupPositions(jmg_, &tmp_jmg_positions_[0]); // TODO this is inefficient
    for (std::size_t joint_id = 0; joint_id < jmg_->getVariableCount(); ++joint_id) // for each joint
    {
      tmp_joint_positions_[joint_id][i] = tmp_jmg_positions_[joint_id];
    }

    tmp_timestamps_[i] = i + 1.0; // dummy value TODO(davetcoleman): is this ok?
  }

  // Setup Spline Fitting
  spline_fitting_.setJointPositions(tmp_joint_positions_, tmp_timestamps_);
}

void MoveItTopp::readPPTrajFromFile(const std::string& filename, TOPP::Trajectory &trajectory)
{
  ROS_INFO_STREAM_NAMED(name_, "Reading PP from filename: " << filename);

  std::string trajectory_string;
  std::ifstream filehandle(filename.c_str());

  filehandle.seekg(0, std::ios::end);
  trajectory_string.reserve(filehandle.tellg());
  filehandle.seekg(0, std::ios::beg);

  trajectory_string.assign((std::istreambuf_iterator<char>(filehandle)),
                           std::istreambuf_iterator<char>());

  // Convert to trajectory
  trajectory.InitFromString(trajectory_string);
}

void MoveItTopp::optimizeTrajectory(const TOPP::Trajectory &old_trajectory, TOPP::Trajectory &new_trajectory)
{
  // Benchmark runtime
  ros::Time start_time = ros::Time::now();

  ROS_INFO_STREAM_NAMED(name_, "Optimizing tractory with " << old_trajectory.dimension << " dims, "
                        << old_trajectory.chunkslist.size() << " waypoints");

  // Error check
  if (!pconstraints_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Constraints object not initialized");
    exit(-1);
  }

  pconstraints_->trajectory = old_trajectory;

  // Set default private tuning parameters
  pconstraints_->bisectionprecision = 0.01;
  pconstraints_->loweringcoef = 0.95;

  // Run TOPP
  ROS_INFO_STREAM_NAMED(name_, "Computing profiles");
  TOPP::dReal sdbeg = 0;
  TOPP::dReal sdend= 0;
  pconstraints_->integrationtimestep = 0;
  pconstraints_->passswitchpointnsteps = 5;
  pconstraints_->extrareps = 0;
  pconstraints_->stepthresh = 0.01;
  ros::Time start_time2 = ros::Time::now();
  TOPP::ComputeProfiles(*pconstraints_, sdbeg, sdend);

  // Benchmark runtime
  double duration2 = (ros::Time::now() - start_time2).toSec();
  ROS_INFO_STREAM_NAMED(name_, "Profiles time: " << duration2 << " seconds (" << 1.0/duration2 << " hz)");

  // Reparameterize
  ROS_INFO_STREAM_NAMED(name_, "Parameterizing");
  pconstraints_->reparamtimestep = 0; // Original value - sets automatically
  //pconstraints_->reparamtimestep = 0.01; // DTC Causes less points to be created, faster
  ros::Time start_time3 = ros::Time::now();
  pconstraints_->trajectory.Reparameterize(*pconstraints_, new_trajectory);

  // Benchmark runtime
  double duration3 = (ros::Time::now() - start_time3).toSec();
  ROS_INFO_STREAM_NAMED(name_, "Parameterize time: " << duration3 << " seconds (" << 1.0/duration3 << " hz)");

  // Benchmark runtime
  double duration = (ros::Time::now() - start_time).toSec();
  ROS_WARN_STREAM_NAMED(name_, "Total time: " << duration << " seconds (" << 1.0/duration << " hz)");
}

bool MoveItTopp::writeTrajectoryToFile(const std::string &file_path, const TOPP::Trajectory &trajectory)
{
  ROS_INFO_STREAM_NAMED(name_, "Writing discretized trajectory to file");

  std::ofstream output_handle;
  output_handle.open(file_path.c_str());

  // Output header -------------------------------------------------------
  output_handle << "time_from_start,";
  for (std::size_t i = 0; i < std::size_t(trajectory.dimension); ++i)
  {
    output_handle << "j" << std::to_string(i) << "_pos,";
    output_handle << "j" << std::to_string(i) << "_vel,";
    output_handle << "j" << std::to_string(i) << "_acc,";
  }
  output_handle << std::endl;

  // Debug
  ROS_INFO_STREAM_NAMED(name_, " - Number of waypoints: " << trajectory.chunkslist.size());
  ROS_INFO_STREAM_NAMED(name_, " - Trajectory duration: " << trajectory.duration);
  ROS_INFO_STREAM_NAMED(name_, " - Trajectory dimension: " << trajectory.dimension);

  // Discretize back into waypoints
  double dt = 0.01;

  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  position.resize(trajectory.dimension);
  velocity.resize(trajectory.dimension);
  acceleration.resize(trajectory.dimension);

  for (double time = 0; time < trajectory.duration; time+=dt)
  {
    trajectory.Eval(time, position);
    trajectory.Evald(time, velocity);
    trajectory.Evaldd(time, acceleration);

    output_handle.precision(10);
    output_handle << time << ",";
    for (std::size_t j = 0; j < position.size(); ++j)
    {
      output_handle << position[j] << ", ";
      output_handle << velocity[j] << ", ";
      output_handle << acceleration[j] << ", ";
    }
    output_handle << std::endl;
  }


  output_handle.close();
  ROS_INFO_STREAM_NAMED(name_, "Saved trajectory to " << file_path);
  return true;
}

bool MoveItTopp::convertTrajToMoveItTraj(robot_trajectory::RobotTrajectory& robot_traj, const TOPP::Trajectory &trajectory)
{
  ROS_INFO_STREAM_NAMED(name_, "Converting TOPP PP trajectory to MoveIt! discretized trajectory");

  // Delete previous trajectory
  robot_traj.clear();

  // Debug
  ROS_INFO_STREAM_NAMED(name_, " - Number of waypoints: " << trajectory.chunkslist.size());
  ROS_INFO_STREAM_NAMED(name_, " - Trajectory duration: " << trajectory.duration);
  ROS_INFO_STREAM_NAMED(name_, " - Trajectory dimension: " << trajectory.dimension);

  // Discretize back into waypoints
  double dt = 0.05;

  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  position.resize(trajectory.dimension);
  velocity.resize(trajectory.dimension);
  acceleration.resize(trajectory.dimension);

  // Re-usable robot state
  robot_state::RobotState state(robot_traj.getRobotModel());

  // Loop through trajectory at discretization dt
  for (double time = 0; time <= trajectory.duration; time += dt)
  {
    // Evaluate PP at this point in time
    trajectory.Eval(time, position);
    trajectory.Evald(time, velocity);
    //trajectory.Evaldd(time, acceleration);

    // Copy to robot state
    state.setJointGroupPositions(jmg_, position);
    state.setJointGroupVelocities(jmg_, velocity);
    //state.setJointGroupAccelerations(jmg_, acceleration);

    // Add state to trajectory
    robot_traj.addSuffixWayPoint(state, dt);
  }

  return true;
}

} // namespace moveit_topp
