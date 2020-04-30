/**
 *  \file       dingo_hardware.cpp
 *  \brief      Class representing Dingo hardware
 *  \copyright  Copyright (c) 2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */

#include <vector>
#include "boost/assign.hpp"
#include "boost/shared_ptr.hpp"
#include "dingo_base/dingo_hardware.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/multi_driver_node.h"

namespace dingo_base
{

DingoHardware::DingoHardware(DingoType type, ros::NodeHandle& nh,
    ros::NodeHandle& pnh, puma_motor_driver::Gateway& gateway):
  dingo_type_(type),
  nh_(nh),
  pnh_(pnh),
  gateway_(gateway),
  active_(false)
{
  pnh_.param<double>("gear_ratio", gear_ratio_, 34.97);  // TODO(tbaltovski): to update
  pnh_.param<int>("encoder_cpr", encoder_cpr_, 1024);    // TODO(tbaltovski): to update

  // Set up the wheels: differs for Dingo-D vs Dingo-O
  ros::V_string joint_names;
  std::vector<uint8_t> joint_canids;
  std::vector<float> joint_directions;
  switch (dingo_type_)
  {
    case DingoType::DINGO_D:
      // TODO(tbaltovski): to check these CAN IDs and directions
      joint_names.assign({"left_wheel", "right_wheel"});  // NOLINT(whitespace/braces)
      joint_canids.assign({2, 3});       // NOLINT(whitespace/braces)
      joint_directions.assign({-1, 1});  // NOLINT(whitespace/braces)
      break;
    case DingoType::DINGO_O:
      // TODO(tbaltovski): to check these CAN IDs and directions
      joint_names.assign({"front_left_wheel", "front_right_wheel",  // NOLINT(whitespace/braces)
          "rear_left_wheel", "rear_right_wheel"});                  // NOLINT(whitespace/braces)
      joint_canids.assign({5, 4, 2, 3});        // NOLINT(whitespace/braces)
      joint_directions.assign({-1, 1, -1, 1});  // NOLINT(whitespace/braces)
      break;
  }

  for (uint8_t i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);

    puma_motor_driver::Driver driver(gateway_, joint_canids[i], joint_names[i]);
    driver.clearStatusCache();
    driver.setEncoderCPR(encoder_cpr_);
    driver.setGearRatio(gear_ratio_ * joint_directions[i]);
    driver.setMode(puma_motor_msgs::Status::MODE_SPEED, 0.1, 0.01, 0.0);  // TODO(tbaltovski): to check
    drivers_.push_back(driver);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode(nh_, drivers_));
}

void DingoHardware::init()
{
  while (!connectIfNotConnected())
  {
    ros::Duration(1.0).sleep();
  }
}

bool DingoHardware::connectIfNotConnected()
{
  if (!gateway_.isConnected())
  {
    if (!gateway_.connect())
    {
      ROS_ERROR("Error connecting to motor driver gateway. Retrying in 1 second.");
      return false;
    }
    else
    {
      ROS_INFO("Connection to motor driver gateway successful.");
    }
  }
  return true;
}

void DingoHardware::configure()
{
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.configureParams();
  }
}

void DingoHardware::verify()
{
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.verifyParams();
  }
}

bool DingoHardware::isActive()
{
  switch (dingo_type_)
  {
    case DingoType::DINGO_D:
      if (active_ == false
         && drivers_[0].isConfigured() == true
         && drivers_[1].isConfigured() == true)
      {
        active_ = true;
        multi_driver_node_->activePublishers(active_);
      }
      else if ((drivers_[0].isConfigured() == false
        || drivers_[1].isConfigured() == false)
        && active_ == true)
      {
        active_ = false;
      }
      break;
    case DingoType::DINGO_O:
      if (active_ == false
         && drivers_[0].isConfigured() == true
         && drivers_[1].isConfigured() == true
         && drivers_[2].isConfigured() == true
         && drivers_[3].isConfigured() == true)
      {
        active_ = true;
        multi_driver_node_->activePublishers(active_);
      }
      else if ((drivers_[0].isConfigured() == false
        || drivers_[1].isConfigured() == false
        || drivers_[2].isConfigured() == false
        || drivers_[3].isConfigured() == false)
        && active_ == true)
      {
        active_ = false;
      }
      break;
  }

  if (active_)
  {
    ROS_INFO("Dingo Hardware Active.");
  }
  else
  {
    ROS_WARN("Dingo Hardware Inactive.");
  }

  return active_;
}

void DingoHardware::powerHasNotReset()
{
  // Checks to see if power flag has been reset for each driver
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    if (driver.lastPower() != 0)
    {
      active_ = false;
      ROS_WARN("There was a power reset on Dev: %d, will reconfigure all drivers.", driver.deviceNumber());
      multi_driver_node_->activePublishers(active_);
      BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
      {
        driver.resetConfiguration();
      }
    }
  }
}

bool DingoHardware::inReset()
{
  return !active_;
}

void DingoHardware::requestData()
{
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.requestFeedbackPowerState();
  }
}

void DingoHardware::updateJointsFromHardware()
{
  uint8_t index = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    Joint* f = &joints_[index];
    f->effort = driver.lastCurrent();
    f->position = driver.lastPosition();
    f->velocity = driver.lastSpeed();
    index++;
  }
}

void DingoHardware::command()
{
  uint8_t i = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.commandSpeed(joints_[i].velocity_command);
    i++;
  }
}

void DingoHardware::canRead()
{
  puma_motor_driver::Message recv_msg;
  while (gateway_.recv(&recv_msg))
  {
    BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
    {
      driver.processMessage(recv_msg);
    }
  }
}

void DingoHardware::canSend()
{
  gateway_.sendAllQueued();
}

}  // namespace dingo_base
