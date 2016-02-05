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

// this package
#include <moveit_topp/spline_fitting.h>

namespace moveit_topp
{

  SplineFitting::SplineFitting()
  {
    // Load rosparams
    // ros::NodeHandle rpnh(nh_, name_);
    // std::size_t error = 0;
    // error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    // rosparam_shortcuts::shutdownIfError(name_, error);

    ROS_INFO_STREAM_NAMED(name_, "SplineFitting Ready.");
  }

  void SplineFitting::readJointTrajFromFile(const std::string& filename, std::size_t num_joints)
  {
    num_joints_ = num_joints;

    // Open file
    std::ifstream input_file;
    input_file.open(filename.c_str());

    // Settings
    std::string line;
    std::string cell;

    // Store data by joints, THEN waypoints
    joint_positions_.resize(num_joints_);

    // Skip header
    std::getline(input_file, line);

    bool first_row_read_file = true;
    double first_timestamp;

    // For each row/trajectory waypoint
    while (std::getline(input_file, line))
    {
      std::stringstream lineStream(line);

      // TIME FROM START
      if (!std::getline(lineStream, cell, ','))
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no time value");
      double timestamp = atof(cell.c_str());
      // Make first timestamp zero
      if (first_row_read_file)
      {
        first_timestamp = timestamp;
        first_row_read_file = false;
      }
      timestamps_.push_back(timestamp - first_timestamp);

      // For each joint
      for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
      {
        // POSITION
        if (!std::getline(lineStream, cell, ','))
          ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
        joint_positions_[joint_id].push_back(atof(cell.c_str()));

        // VELOCITY
        if (!std::getline(lineStream, cell, ','))
          ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
        // UNUSED

        // ACCELERATION
        if (!std::getline(lineStream, cell, ','))
          ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
        // UNUSED
      }
    }  // while
  }

  void SplineFitting::setJointPositions(const std::vector<std::vector<double> >& joint_positions,
                                        const std::vector<double>& timetamps)
  {
    // Error check
    if (joint_positions.empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "No waypoints passed in");
      return;
    }
    if (joint_positions.front().empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "No joint values passed in");
      return;
    }
    // Copy in num joints
    num_joints_ = joint_positions.size();

    // Save joint positions
    joint_positions_ = joint_positions;
    timestamps_ = timetamps;
  }

  void SplineFitting::fitSpline()
  {
    ROS_INFO_STREAM_NAMED(name_, "Fitting spline");

    // Error check
    if (joint_positions_.empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "No data loaded to fit");
      return;
    }
    if (joint_positions_.size() != num_joints_)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Incorrect number of joints passed in " << joint_positions_.size());
      return;
    }
    if (timestamps_.size() != joint_positions_.front().size())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Data size invalid between timestamps and joint_positions");
      return;
    }

    // Get the velocities
    bool use_simple_derivative = false;
    if (use_simple_derivative)
    {
      calcSimpleDerivative();

      // Error check
      if (timestamps_.size() != joint_velocities_.front().size())
      {
        ROS_ERROR_STREAM_NAMED(name_, "Data size invalid between timestamps and joint_velocities");
        return;
      }
    }

    int ndata = timestamps_.size();
    double* tdata = &timestamps_[0];
    coefficients_.resize(num_joints_);

    // Benchmark runtime
    ros::Time start_time = ros::Time::now();

    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      double* ydata = &joint_positions_[joint_id][0];
      double* ypdata;
      if (use_simple_derivative)
        ypdata = &joint_velocities_[joint_id][0];
      else
        ypdata = new double[ndata];

      // Set derivatives for a piecewise cubic Hermite interpolant.
      ROS_INFO_STREAM_NAMED(name_, "Calculating spline pchip set");
      spline_pchip_set(ndata, tdata, ydata, ypdata);

      // Set up a piecewise cubic Hermite interpolant
      ROS_INFO_STREAM_NAMED(name_, "Calculating spline hermite set");
      coefficients_[joint_id] = spline_hermite_set(ndata, tdata, ydata, ypdata);

      // Output coefficients
      bool verbose = false;
      if (verbose)
      {
        std::cout << "Timestamp       Position        ";
        if (use_simple_derivative)
          std::cout << "Velocity       ";
        std::cout << "SVelocity        Coefficient1    Coefficient2    Coefficient3    Coefficient4 " << std::endl;
        for (std::size_t j = 0; j < std::size_t(ndata); ++j)
        {
          std::cout << std::fixed << tdata[j] << "\t" << ydata[j] << "\t";
          if (use_simple_derivative)
            std::cout << joint_velocities_[joint_id][j] << "\t";
          std::cout << ypdata[j] << "\t";
          for (std::size_t i = 0; i < 4; ++i)
          {
            std::cout << coefficients_[joint_id][j * 4 + i] << "\t";
          }
          std::cout << std::endl;
        }
      }
    }  // end for

    // Benchmark runtime
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED(name_, "Conversion to polynomial: " << duration << " seconds");
  }

  void SplineFitting::writeCoefficientsToFile(const std::string& file_path)
  {
    ROS_INFO_STREAM_NAMED(name_, "Writing coefficients to file");

    std::ofstream output_handle;
    output_handle.open(file_path.c_str());

    // Output header -------------------------------------------------------
    output_handle << "duration,C1,C2,C3,C4" << std::endl;

    // TODO(davetcoleman): currently only outputs first joint
    std::size_t joint_id = 0;

    for (std::size_t i = 0; i < timestamps_.size() - 1; ++i)
    {
      output_handle.precision(10);
      // output_handle << timestamps_[i+1] - timestamps_[i] << ",";
      output_handle << timestamps_[i] << ",";

      for (std::size_t c = 0; c < 4; ++c)
      {
        output_handle << coefficients_[joint_id][i * 4 + (3 - c)] << ",";
      }
      output_handle << std::endl;
    }
    output_handle.close();
    ROS_INFO_STREAM_NAMED(name_, "Saved trajectory to file " << file_path);
  }

  void SplineFitting::getPPTrajectory(TOPP::Trajectory& trajectory)
  {
    ROS_INFO_STREAM_NAMED(name_, "Converting coefficients to new format");

    std::list<TOPP::Chunk> chunks_list(timestamps_.size());
    std::list<TOPP::Chunk>::iterator chunk_it = chunks_list.begin();
    std::vector<TOPP::Polynomial> polynomials_vector(num_joints_);
    std::vector<TOPP::dReal> coefficients_vector(4);

    for (std::size_t chunk_id = 0; chunk_id < timestamps_.size() - 1; ++chunk_id) // TODO(davetcoleman): why is it minus one?
    {
      for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
      {
        for (std::size_t coeff_id = 0; coeff_id < 4; ++coeff_id)
        {
          coefficients_vector[coeff_id] = coefficients_[joint_id][chunk_id * 4 + coeff_id];
        }
        polynomials_vector[joint_id] = TOPP::Polynomial(coefficients_vector);
      }

      double duration = timestamps_[chunk_id + 1] - timestamps_[chunk_id];
      //chunks_list[chunk_id] = TOPP::Chunk(duration, polynomials_vector);
      (*chunk_it) = TOPP::Chunk(duration, polynomials_vector);
      chunk_it++;
    }

    // Create final trajectory
    trajectory.InitFromChunksList(chunks_list);
  }

  void SplineFitting::calcSimpleDerivative()
  {
    ROS_INFO_STREAM_NAMED(name_, "Calculating simple derivatives");
    joint_velocities_.resize(num_joints_);

    bool verbose = false;

    // For each joint
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      // For each waypoint
      for (std::size_t i = 1; i < joint_positions_[joint_id].size(); ++i)
      {
        double pos_diff = joint_positions_[joint_id][i] - joint_positions_[joint_id][i - 1];
        double t_diff = timestamps_[i] - timestamps_[i - 1];
        double vel = pos_diff / t_diff;
        if (verbose)
          std::cout << "joint_id: " << joint_id << " i: " << std::setfill('0') << std::setw(2) << i
                    << " pos_diff: " << std::fixed << pos_diff << " \tt_diff: " << t_diff << " \tvel: " << vel
                    << std::endl;

        joint_velocities_[joint_id].push_back(vel);
      }
      joint_velocities_[joint_id].push_back(0.0);  // TODO(davetcoleman): how to calculate final derivative?
    }
  }

}  // namespace moveit_topp
