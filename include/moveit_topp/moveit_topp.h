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
#include "TOPP.h"
#include "KinematicLimits.h"
#include "TorqueLimits.h"
#include "PolygonConstraints.h"

// Spline
#include "spline.hpp"

// MoveIt
#include <moveit_msgs/RobotTrajectory.h>

namespace moveit_topp
{

class MoveItTopp
{
public:

  /**
   * \brief Constructor
   */
  MoveItTopp()
  {
    // Load rosparams
    //ros::NodeHandle rpnh(nh_, name_);
    //std::size_t error = 0;
    //error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    //rosparam_shortcuts::shutdownIfError(name_, error);
    ROS_INFO_STREAM_NAMED(name_,"MoveItTopp Ready.");
  }

  void readPPTrajFromFile(const std::string& filename, TOPP::Trajectory &trajectory)
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

  void optimizeTrajectory(const TOPP::Trajectory &old_trajectory, TOPP::Trajectory &new_trajectory)
  {
    ROS_INFO_STREAM_NAMED(name_, "Optimizing tractory with " << old_trajectory.dimension << " dims, "
                          << old_trajectory.chunkslist.size() << " waypoints");
    //moveit_msgs::RobotTrajectory &trajectory_msg;

    // Config TODO(davetcoleman): do not hard code
    std::size_t num_joints = 7;
    double discrtimestep = 0.005;
    double max_velocity = 2;
    double max_acceleration = 10;
    std::string constraint_string = std::to_string(discrtimestep);
    constraint_string.append("\n");
    // TODO(davetcoleman): do not hardcode constraints
    std::string temp = std::to_string(max_velocity).append(" ");
    for (std::size_t i = 0; i < num_joints; ++i)
      constraint_string.append(temp); // velocity
    constraint_string.append("\n");
    temp = std::to_string(max_acceleration).append(" ");
    for (std::size_t i = 0; i < num_joints; ++i)
      constraint_string.append(temp); // velocity

    // Debug
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "constraint_string: \n" << constraint_string << std::endl;

    // Setup constraints
    pconstraints_.reset(new TOPP::KinematicLimits(constraint_string));
    pconstraints_->trajectory = old_trajectory;

    // Set default private tuning parameters
    pconstraints_->bisectionprecision = 0.01;
    pconstraints_->loweringcoef = 0.95;

    // Run TOPP
    ROS_INFO_STREAM_NAMED(name_, "Computing profiles");
    RunComputeProfiles(0, 0);

    // Reparameterize
    ROS_INFO_STREAM_NAMED(name_, "Parameterizing");
    pconstraints_->reparamtimestep = 0;
    pconstraints_->trajectory.Reparameterize(*pconstraints_, new_trajectory);

    // Get results
    //WriteResultTrajectory();
    //std::cout << "new_trajectory_string_: " << new_trajectory_string_ << std::endl;
  }

  bool writeTrajectoryToFile(const std::string &file_path, const TOPP::Trajectory &trajectory)
  {
    ROS_INFO_STREAM_NAMED(name_, "Writing discretized trajectory to file");

    std::ofstream output_handle;
    output_handle.open(file_path.c_str());

    // Output header -------------------------------------------------------
    output_handle << "time_from_start,j0,j1,j2,j3,j4,j5,j6" << std::endl;

    // Debug
    ROS_INFO_STREAM_NAMED(name_, " - Number of waypoints: " << trajectory.chunkslist.size());
    ROS_INFO_STREAM_NAMED(name_, " - Trajectory duration: " << trajectory.duration);

    // Discretize back into waypoints
    double dt = 0.01;

    std::vector<double> output;
    output.resize(trajectory.dimension);

    for (double time = 0; time < trajectory.duration; time+=dt)
    {
      trajectory.Eval(time, output);

      output_handle.precision(10);
      output_handle << time << ",";
      for (std::size_t j = 0; j < output.size(); ++j)
      {
        output_handle << output[j] << ", ";
      }
      output_handle << std::endl;
    }


    output_handle.close();
    ROS_INFO_STREAM_NAMED(name_, "Saved trajectory to " << file_path);
    return true;
  }

  int RunComputeProfiles(TOPP::dReal sdbeg, TOPP::dReal sdend){
    // Set tuning parameters
    pconstraints_->integrationtimestep = 0;
    pconstraints_->passswitchpointnsteps = 5;
    pconstraints_->extrareps = 0;
    pconstraints_->stepthresh = 0.01;

    int res = TOPP::ComputeProfiles(*pconstraints_, sdbeg, sdend);
    return res;
  }

  //void WriteResultTrajectory(TOPP::Trajectory &trajectory){
    // TODO(davetcoleman): remove - i don't think i need this

    // std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
    //std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
    // printf("WriteResultTrajectory: %d %f %d blah\n",
    //        trajectory.dimension, trajectory.duration,
    //        trajectory.degree);
  //   std::stringstream ss;
  //   ss << std::setprecision(17);
  //   trajectory.Write(ss);
  //   trajectorystring_ = ss.str();
  // }

private:

  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "moveit_topp";

  // A shared node handle
  ros::NodeHandle nh_;

  // TOPP vars
  boost::shared_ptr<TOPP::Constraints> pconstraints_;
}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<MoveItTopp> MoveItToppPtr;
typedef boost::shared_ptr<const MoveItTopp> MoveItToppConstPtr;

} // namespace moveit_topp

#endif  // MOVEIT_TOPP_MOVEIT_TOPP_H
