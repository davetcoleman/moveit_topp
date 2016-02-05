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
   Desc:   Wrapper for John Burkardt's spline funcitonality
*/

#ifndef MOVEIT_TOPP_SPLINE_FITTING_H
#define MOVEIT_TOPP_SPLINE_FITTING_H

// C++
#include <string>
#include <fstream>
#include <streambuf>

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Spline
#include <spline/spline.hpp>

// TOPP
#include <TOPP/src/Trajectory.h>

namespace moveit_topp
{
class SplineFitting
{
public:
  /**
   * \brief Constructor
   */
  SplineFitting();

  void readJointTrajFromFile(const std::string& filename, std::size_t num_joints);

  void setJointPositions(const std::vector<std::vector<double> >& joint_positions,
                         const std::vector<double>& timetamps);

  void fitSpline();

  void writeCoefficientsToFile(const std::string& file_path);

  /**
   * \brief Convert the coefficients into the TOPP trajectory format
   */
  void getPPTrajectory(TOPP::Trajectory& trajectory);

  void calcSimpleDerivative();

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "spline_fitting";

  // A shared node handle
  ros::NodeHandle nh_;

  std::size_t num_joints_;

  // Read data
  std::vector<double> timestamps_;
  std::vector<std::vector<double>> joint_positions_;
  std::vector<std::vector<double>> joint_velocities_;
  std::vector<double*> coefficients_;  // SPLINE_HERMITE_SET

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<SplineFitting> SplineFittingPtr;
typedef boost::shared_ptr<const SplineFitting> SplineFittingConstPtr;

}  // namespace moveit_topp

#endif  // MOVEIT_TOPP_SPLINE_FITTING_H
