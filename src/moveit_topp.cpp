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

namespace moveit_topp
{

class MoveItTopp
{
public:

  /**
   * \brief Constructor
   */
  MoveItTopp()
    : name_("moveit_topp")
  {
    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    //error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    rosparam_shortcuts::shutdownIfError(name_, error);

    std::string trajectory_string;
    readTrajectoryFromFile("/home/dave/ros/current/ws_acme/src/moveit_topp/external/TOPP/tests/matlab_traj.csv",
                           trajectory_string);

    // Load trajectory
    TOPP::Trajectory* ptrajectory = new TOPP::Trajectory(trajectory_string);

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
    pconstraints_->trajectory = *ptrajectory;

    // Set default public tuning parameters
    integrationtimestep = 0;
    reparamtimestep = 0;
    passswitchpointnsteps = 5;
    extrareps = 0;

    // Set default private tuning parameters
    pconstraints_->bisectionprecision = 0.01;
    pconstraints_->loweringcoef = 0.95;

    // Run TOPP
    RunComputeProfiles(0, 0);
    ReparameterizeTrajectory();

    // Get results
    //WriteResultTrajectory();
    //std::cout << "new_trajectory_string_: " << new_trajectory_string_ << std::endl;

    // Write discretized trajectory to file
    writeTrajectoryToFile("/home/dave/ros/current/ws_acme/src/moveit_topp/external/TOPP/tests/matlab_traj_output.csv");

    ROS_INFO_STREAM_NAMED(name_,"MoveItTopp Ready.");
  }

  void readTrajectoryFromFile(const std::string& filename, std::string& output)
  {
    std::cout << "filename " << filename << std::endl;
    std::ifstream filehandle(filename.c_str());

    filehandle.seekg(0, std::ios::end);
    output.reserve(filehandle.tellg());
    filehandle.seekg(0, std::ios::beg);

    output.assign((std::istreambuf_iterator<char>(filehandle)),
                  std::istreambuf_iterator<char>());
  }

  bool writeTrajectoryToFile(const std::string &file_path)
  {
    std::ofstream output_handle;
    output_handle.open(file_path.c_str());

    // Output header -------------------------------------------------------
    output_handle << "time_from_start,j0,j1,j2,j3,j4,j5,j6";
    output_handle << std::endl;

    // Debug
    ROS_INFO_STREAM_NAMED(name_, "Number of chunks: " << "size: " << new_trajectory_.chunkslist.size());

    // Discretize back into waypoints
    double dt = 0.01;

    /*
    double time = 0;
    std::list<TOPP::Chunk>::iterator itchunk = new_trajectory_.chunkslist.begin();
    for(;itchunk != new_trajectory_.chunkslist.end(); itchunk++)
    {
      output_handle.precision(10);
      output_handle << time << ",";
      for (std::size_t j = 0; j < itchunk->polynomialsvector.size(); ++j) // polynomial ~= dimension
      {
        // TODO(davetcoleman): for duration of chunk
        double value = itchunk->polynomialsvector[j].Eval(time);
        output_handle << value << ", ";
      }
      time += dt;
      output_handle << std::endl;
    }
    */
    std::vector<double> output;
    output.resize(new_trajectory_.dimension);

    for (double time = 0; time < new_trajectory_.duration; time+=dt)
    {
      new_trajectory_.Eval(time, output);

      output_handle.precision(10);
      output_handle << time << ",";
      for (std::size_t j = 0; j < output.size(); ++j)
      {
        output_handle << output[j] << ", ";
      }
      output_handle << std::endl;
    }


    output_handle.close();
    ROS_INFO_STREAM_NAMED(name_, "Saved trajectory to file " << file_path);
    return true;
  }

  int RunComputeProfiles(TOPP::dReal sdbeg, TOPP::dReal sdend){
    // Set tuning parameters
    pconstraints_->integrationtimestep = integrationtimestep;
    pconstraints_->passswitchpointnsteps = passswitchpointnsteps;
    pconstraints_->extrareps = extrareps;
    pconstraints_->stepthresh = 0.01;

    int res = TOPP::ComputeProfiles(*pconstraints_,sdbeg,sdend);
    resduration = pconstraints_->resduration;
    return res;
  }

  int ReparameterizeTrajectory(TOPP::dReal reparamtimestep=0)
  {
    // Set tuning parameters
    pconstraints_->reparamtimestep = reparamtimestep;

    int ret = pconstraints_->trajectory.Reparameterize(*pconstraints_, new_trajectory_);
    return ret;
  }

  void WriteResultTrajectory(){
    // TODO(davetcoleman): remove - i don't think i need this

    // std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
    //std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
    // printf("WriteResultTrajectory: %d %f %d blah\n",
    //        new_trajectory_.dimension, new_trajectory_.duration,
    //        new_trajectory_.degree);
    std::stringstream ss;
    ss << std::setprecision(17);
    new_trajectory_.Write(ss);
    new_trajectory_string_ = ss.str();
  }

private:

  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // TOPP vars
  boost::shared_ptr<TOPP::Constraints> pconstraints_;

  TOPP::Trajectory new_trajectory_;
  std::string new_trajectory_string_;

  int ntangenttreated;
  int nsingulartreated;
  TOPP::dReal resduration;
  TOPP::dReal sdendmin, sdendmax;
  TOPP::dReal sdbegmin, sdbegmax;

  // Tuning parameters
  TOPP::dReal integrationtimestep, reparamtimestep;
  int passswitchpointnsteps, extrareps;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<MoveItTopp> MoveItToppPtr;
typedef boost::shared_ptr<const MoveItTopp> MoveItToppConstPtr;

} // namespace moveit_topp

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "moveit_topp");
  ROS_INFO_STREAM_NAMED("main", "Starting MoveItTopp...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  moveit_topp::MoveItTopp server;

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif  // MOVEIT_TOPP_MOVEIT_TOPP_H
