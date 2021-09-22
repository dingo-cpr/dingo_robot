/**
 *  \file       dingo_cooling.cpp
 *  \brief      Cooling control class for Dingo
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

#include <math.h>

#include "dingo_base/dingo_cooling.hpp"

namespace dingo_base
{

const double DingoCooling::LINEAR_VEL_THRESHOLD = 0.1;    // m/s
const double DingoCooling::ANGULAR_VEL_THRESHOLD = 0.4;   // rad/s
const double DingoCooling::MOTION_COMMAND_TIMEOUT = 3.0;  // seconds

DingoCooling::DingoCooling(std::string node_name) : Node(node_name)
{
  cmd_fans_pub_ = this->create_publisher<dingo_msgs::msg::Fans>("mcu/cmd_fans", 10);
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("dingo_velocity_controller/cmd_vel", 10, std::bind(&DingoCooling::cmdVelCallback, this, std::placeholders::_1));
  cmd_fans_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DingoCooling::cmdFansCallback, this, std::placeholders::_1));
  cmd_fans_msg_.fan = dingo_msgs::msg::Fans::FAN_ON_LOW;
}

void DingoCooling::cmdVelCallback(geometry_msgs::msg::Twist::ConstSharedPtr twist)
{
  if (fabs(twist->linear.x) >= LINEAR_VEL_THRESHOLD ||
      fabs(twist->linear.y) >= LINEAR_VEL_THRESHOLD ||
      fabs(twist->angular.z) >= ANGULAR_VEL_THRESHOLD)
  {
    cmd_fans_msg_.fan = dingo_msgs::msg::Fans::FAN_ON_HIGH;
  }
  last_motion_cmd_time_ = this->get_clock()->now().seconds();
}

void DingoCooling::cmdFansCallback()
{
  if (this->get_clock()->now().seconds() - last_motion_cmd_time_ > MOTION_COMMAND_TIMEOUT)
  {
    cmd_fans_msg_.fan = dingo_msgs::msg::Fans::FAN_ON_LOW;
  }
  cmd_fans_pub_->publish(cmd_fans_msg_);
}

}  // namespace dingo_base
