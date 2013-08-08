/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <effort_controllers/joint_velocity_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>


namespace effort_controllers {

JointVelocityController::JointVelocityController()
  : command_(0),
    loop_count_(0),
    in_position_mode_(false)
{}

JointVelocityController::~JointVelocityController()
{
  sub_command_.shutdown();
}

bool JointVelocityController::init(hardware_interface::EffortJointInterface *robot, 
  const std::string &joint_name, const control_toolbox::Pid &pid)
{
  pid_controller_ = pid;

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  return true;
}

bool JointVelocityController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
    (n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointVelocityController::setCommandCB, this);

  return true;
}


void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);

}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

void JointVelocityController::printDebug()
{
  pid_controller_.printValues();
}

std::string JointVelocityController::getJointName()
{
  return joint_.getName();
}

// Set the joint velocity command
void JointVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void JointVelocityController::getCommand(double& cmd)
{
  cmd = command_;
}

void JointVelocityController::starting(const ros::Time& time)
{
  command_ = 0.0;
  pid_controller_.reset();
}

void JointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  double error, vel_error;
  double commanded_effort;

  // If the velocity command is zero, assume we want to maintain position
  if( command_ == 0 )
  {
    if( in_position_mode_ == false )
    {
      // Remember the current position so that we can maintain it in the future
      position_mode_command_ = joint_.getPosition();
      in_position_mode_ = true;
      //ROS_DEBUG_STREAM_NAMED("update","Switching velocity controller to position mode with setpoint " 
      //  << position_mode_command_);
    }
  }
  else if( in_position_mode_ == true )
  {
    // switch back to velocity mode
    in_position_mode_ = false;
    //ROS_DEBUG_STREAM_NAMED("update","Switching velocity controller back to velocity mode");
  }

  // Send position command if appropriate
  if( in_position_mode_ )
  {
    double current_position = joint_.getPosition();

    // Make sure joint is within limits if applicable
    enforceJointLimits(position_mode_command_);

    // Compute position error
    if (joint_urdf_->type == urdf::Joint::REVOLUTE)
    {
      angles::shortest_angular_distance_with_limits(
        current_position,
        position_mode_command_,
        joint_urdf_->limits->lower,
        joint_urdf_->limits->upper,
        error);
    }
    else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
    {
      error = angles::shortest_angular_distance(current_position, position_mode_command_);
    }
    else if (joint_urdf_->type == urdf::Joint::PRISMATIC)
    {
      error = position_mode_command_ - current_position;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("update","Unknown joint type " << joint_urdf_->type);
    }

    // Compute velocity error. By default we assume desired velocity is 0 (velocity is not required)
    vel_error = 0 - joint_.getVelocity();

    // Set the PID error and compute the PID command with nonuniform
    // time step size. This also allows the user to pass in a precomputed derivative error.
    commanded_effort = pid_controller_.computeCommand(error, vel_error, period);    
  }
  else // in velocity mode
  {
    error = command_ - joint_.getVelocity();

    // Set the PID error and compute the PID command with nonuniform time
    // step size. The derivative error is computed from the change in the error
    // and the timestep dt.
    commanded_effort = pid_controller_.computeCommand(error, period);
  }

  joint_.setCommand(commanded_effort);

  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void JointVelocityController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void JointVelocityController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace


PLUGINLIB_EXPORT_CLASS( effort_controllers::JointVelocityController, controller_interface::ControllerBase)
