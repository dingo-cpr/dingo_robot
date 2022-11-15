/**
 *  \file       dingo_lighting.cpp
 *  \brief      Lighting control class for Dingo
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

#include "dingo_base/dingo_lighting.h"
#include "dingo_base/dingo_power_levels.h"

namespace dingo_base
{

/** Contains the colour definitions for the corner lights */
namespace Colours
{
  static const uint32_t Off = 0x000000;
  static const uint32_t Red_H = 0xFF0000;
  static const uint32_t Red_M = 0xAA00000;
  static const uint32_t Red_L = 0x550000;
  static const uint32_t Green_H = 0x00FF00;
  static const uint32_t Green_M = 0x00AA00;
  static const uint32_t Green_L = 0x001100;
  static const uint32_t Blue_H = 0x0000FF;
  static const uint32_t Blue_M = 0x0000AA;
  static const uint32_t Blue_L = 0x000011;
  static const uint32_t Yellow_H = 0xFFFF00;
  static const uint32_t Yellow_M = 0xAAAA00;
  static const uint32_t Yellow_L = 0x111100;
  static const uint32_t Orange_H = 0xFF8C00;
  static const uint32_t Orange_M = 0xAA5D00;
  static const uint32_t Orange_L = 0x552D00;
  static const uint32_t White_H = 0xFFFFFF;
  static const uint32_t White_M = 0xAAAAAA;
  static const uint32_t White_L = 0x111111;
}  // namespace Colours

DingoLighting::DingoLighting(ros::NodeHandle* nh) :
  nh_(nh),
  pub_period_(1.0/20),
  allow_user_(false),
  user_publishing_(false),
  state_(State::Idle),
  current_pattern_count_(0)
{
  lights_pub_ = nh_->advertise<dingo_msgs::Lights>("mcu/cmd_lights", 1);

  user_cmds_sub_ = nh_->subscribe("cmd_lights", 1, &DingoLighting::userCmdCallback, this);
  mcu_status_sub_ = nh_->subscribe("mcu/status", 1, &DingoLighting::mcuStatusCallback, this);
  battery_state_sub_ = nh_->subscribe("battery/status", 1, &DingoLighting::batteryStateCallback, this);
  puma_status_sub_ = nh_->subscribe("status", 1, &DingoLighting::pumaStatusCallback, this);
  cmd_vel_sub_ = nh_->subscribe("cmd_vel", 1, &DingoLighting::cmdVelCallback, this);

  pub_timer_ = nh_->createTimer(ros::Duration(pub_period_), &DingoLighting::timerCallback, this);
  user_timeout_ = nh_->createTimer(ros::Duration(1.0/1), &DingoLighting::userTimeoutCallback, this);

  using namespace Colours;  // NOLINT(build/namespaces)

  auto off_pattern = pattern{Off, Off, Off, Off};

  patterns_.battery_fault = blinkPattern(
    pattern{Yellow_H, Yellow_H, Yellow_H, Yellow_H},
    pattern{Red_H, Red_H, Red_H, Red_H}, 2.0, 0.5);

  patterns_.shore_power_fault = blinkPattern(
    pattern{Blue_H, Blue_H, Blue_H, Blue_H},
    pattern{Red_H, Red_H, Red_H, Red_H}, 2.0, 0.5);

  patterns_.puma_fault = blinkPattern(
    pattern{Red_H, Red_H, Red_H, Red_H},
    pattern{Red_H, Red_H, Red_H, Red_H}, 1.0, 0.5);

  patterns_.shore_power = pulsePattern(
    pattern{Blue_L, Blue_L, Blue_L, Blue_L},
    pattern{Blue_H, Blue_H, Blue_H, Blue_H}, 4.0);

  patterns_.manual_charging = pulsePattern(
    pattern{Red_L, Red_L, Red_L, Red_L},
    pattern{Red_H, Red_H, Red_H, Red_H}, 4.0);

  patterns_.stopped = blinkPattern(
    pattern{Red_H, Red_H, Red_H, Red_H},
    off_pattern, 2.0, 0.5);

  patterns_.reset = blinkPattern(
    pattern{Off, Red_H, Off, Red_H},
    pattern{Red_H, Off, Red_H, Off}, 2.0, 0.5);

  patterns_.low_battery = pulsePattern(
    pattern{Yellow_L, Yellow_L, Yellow_L, Yellow_L},
    pattern{Yellow_H, Yellow_H, Yellow_H, Yellow_H}, 4.0);

  patterns_.driving = solidPattern(pattern{Red_M, White_M, White_M, Red_M});

  patterns_.idle = solidPattern(pattern{Red_L, White_L, White_L, Red_L});
}

dingo_base::LightsPatterns DingoLighting::solidPattern(dingo_base::pattern pattern)
{
  LightsPatterns lights_pattern;
  lights_pattern.push_back(pattern);
  return lights_pattern;
}

dingo_base::LightsPatterns DingoLighting::blinkPattern(dingo_base::pattern first,
                                                       dingo_base::pattern second,
                                                       double duration,
                                                       double duty_cycle)
{
  LightsPatterns lights_pattern;
  uint32_t steps = static_cast<uint32_t>(duration / pub_period_);

  if (duration <= pub_period_)
  {
    lights_pattern.push_back(first);
  }
  else
  {
    for (uint32_t i = 0; i < steps*duty_cycle; i++)
    {
      lights_pattern.push_back(first);
    }

    for (uint32_t i = 0; i < steps - steps*duty_cycle; i++)
    {
      lights_pattern.push_back(second);
    }
  }

  return lights_pattern;
}

dingo_base::LightsPatterns DingoLighting::pulsePattern(dingo_base::pattern colour_l,
                                                       dingo_base::pattern colour_h,
                                                       double duration)
{
  LightsPatterns lights_pattern;
  std::array<dingo_msgs::RGB, NUM_LEDS> increment;
  uint32_t steps = static_cast<uint32_t>(duration / pub_period_);

  if (duration <= pub_period_)
  {
    lights_pattern.push_back(colour_l);
  }
  else
  {
    dingo_msgs::RGB rgb_l, rgb_h;

    for (uint8_t i=0; i < NUM_LEDS; i++)
    {
      setRGB(&rgb_l, colour_l[i]);
      setRGB(&rgb_h, colour_h[i]);
      increment[i].red = static_cast<uint8_t>(
        std::round(static_cast<double>(rgb_h.red - rgb_l.red) / (steps / 2)));

      increment[i].green = static_cast<uint8_t>(
        std::round(static_cast<double>(rgb_h.green - rgb_l.green) / (steps / 2)));

      increment[i].blue = static_cast<uint8_t>(
        std::round(static_cast<double>(rgb_h.blue - rgb_l.blue) / (steps / 2)));
    }

    for (uint32_t i = 0; i < steps / 2; i++)
    {
      pattern step_pattern;
      for (uint8_t j=0; j < colour_l.size(); j++)
      {
        step_pattern[j] = colour_l[j] + i * (
          increment[j].red << 16 |
          increment[j].green << 8 |
          increment[j].blue);
      }
      lights_pattern.push_back(step_pattern);
    }

    for (uint32_t i = 0; i < steps - steps / 2; i++)
    {
      pattern step_pattern;
      for (uint8_t j=0; j < colour_h.size(); j++)
      {
        step_pattern[j] = colour_h[j] - i * (
          increment[j].red << 16 |
          increment[j].green << 8 |
          increment[j].blue);
      }
      lights_pattern.push_back(step_pattern);
    }
  }
  
  return lights_pattern;
}

void DingoLighting::setRGB(dingo_msgs::RGB* rgb, uint32_t colour)
{
  rgb->red = static_cast<uint8_t>((colour & 0xFF0000) >> 16);
  rgb->green = static_cast<uint8_t>((colour & 0x00FF00) >> 8);
  rgb->blue = static_cast<uint8_t>((colour & 0x0000FF));
}

void DingoLighting::setLights(dingo_msgs::Lights* lights, uint32_t pattern[4])
{
  for (int i = 0; i < 4; i++)
  {
    setRGB(&lights->lights[i], pattern[i]);
  }
}

void DingoLighting::userCmdCallback(const dingo_msgs::Lights::ConstPtr& lights_msg)
{
  // If lighting output is controlled by user (not current state), apply it here
  if (allow_user_)
  {
    lights_pub_.publish(lights_msg);
  }
  user_publishing_ = true;
}

void DingoLighting::mcuStatusCallback(const dingo_msgs::Status::ConstPtr& status_msg)
{
  mcu_status_msg_ = *status_msg;
}

void DingoLighting::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& battery_msg)
{
  battery_state_msg_ = *battery_msg;
}

void DingoLighting::pumaStatusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg)
{
  pumas_status_msg_ = *status_msg;
}

void DingoLighting::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_msg_ = *msg;
}

void DingoLighting::timerCallback(const ros::TimerEvent&)
{
  updateState();

  switch (state_)
  {
    case State::Idle:
    case State::Driving:
    case State::ShorePower:
      allow_user_ = true;
      break;
    case State::LowBattery:
    case State::NeedsReset:
    case State::BatteryFault:
    case State::ShoreFault:
    case State::PumaFault:
    case State::Stopped:
      allow_user_ = false;
      break;
  }

  // If lighting output is controlled by current state, apply it here
  if (!user_publishing_ || !allow_user_)
  {
    dingo_msgs::Lights lights_msg;
    updatePattern();
    setLights(&lights_msg, &current_pattern_[0]);
    lights_pub_.publish(lights_msg);
  }
}

void DingoLighting::userTimeoutCallback(const ros::TimerEvent&)
{
  user_publishing_ = false;
}

void DingoLighting::setState(DingoLighting::State new_state)
{
  if (new_state < state_)
  {
    state_ = new_state;
  }
}

void DingoLighting::updateState()
{
  state_ = State::Idle;
  // Shore power connected
  if (mcu_status_msg_.shore_power_connected == true)
  {
    // Shore power overvoltage detected on MCU, or VSys >= 16.0V
    if (mcu_status_msg_.shore_power_ov ||
        mcu_status_msg_.measured_battery >= dingo_power::SHORE_POWER_OVER_VOLT)
    {
      setState(State::ShoreFault);
    }
    else
    {
      setState(State::ShorePower);
    }
  }
  else
  {
    // SLA battery
    if (battery_state_msg_.power_supply_technology == sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
    {
      if (mcu_status_msg_.measured_battery >= dingo_power::BATTERY_SLA_OVER_VOLT)
      {
        setState(State::BatteryFault);
      }
      else if (mcu_status_msg_.measured_battery <= dingo_power::BATTERY_SLA_LOW_VOLT)
      {
        setState(State::LowBattery);
      }
    }
    else // Li Battery
    {
      if (mcu_status_msg_.measured_battery >= dingo_power::BATTERY_LITHIUM_OVER_VOLT)
      {
        setState(State::BatteryFault);
      }
      else if (battery_state_msg_.percentage <= dingo_power::BATTERY_LITHIUM_LOW_PERCENT)
      {
        setState(State::LowBattery);
      }
    }
  }

  // Puma faults
  pattern puma_indicator = {Colours::Red_H, Colours::Red_H, Colours::Red_H, Colours::Red_H};
  for (size_t i = 0; i < pumas_status_msg_.drivers.size(); i++)
  {
    if (pumas_status_msg_.drivers[i].fault != 0)
    {
      puma_indicator[(i + 1) % NUM_LEDS] = Colours::Off;
      setState(State::PumaFault);
    }
  }

  if (state_ == State::PumaFault)
  {
    patterns_.puma_fault = blinkPattern(
      puma_indicator,
      {Colours::Red_H, Colours::Red_H, Colours::Red_H, Colours::Red_H}, 2.0, 0.5);
  }

  if (mcu_status_msg_.manual_charger_connected) // Manual charger connected
  {
    setState(State::Charging);
  }
  else if (mcu_status_msg_.stop_engaged == true) // E-Stop
  {
    setState(State::Stopped);
  }
  else if (mcu_status_msg_.drivers_active == false) // Needs Reset
  {
    setState(State::NeedsReset);
  }
  else if (cmd_vel_msg_.linear.x != 0.0 ||
           cmd_vel_msg_.linear.y != 0.0 ||
           cmd_vel_msg_.angular.z != 0.0)
  {
    setState(State::Driving);
  }
}

void DingoLighting::updatePattern()
{
  if (old_state_ != state_)
  {
    current_pattern_count_ = 0;
  }

  LightsPatterns new_pattern;

  switch (state_)
  {
    case State::Stopped:
      new_pattern = patterns_.stopped;
      break;
    case State::BatteryFault:
      new_pattern = patterns_.battery_fault;
      break;
    case State::PumaFault:
      new_pattern = patterns_.puma_fault;
      break;
    case State::ShoreFault:
      new_pattern = patterns_.shore_power_fault;
      break;
    case State::NeedsReset:
      new_pattern = patterns_.reset;
      break;
    case State::LowBattery:
      new_pattern = patterns_.low_battery;
      break;
    case State::Driving:
      new_pattern = patterns_.driving;
      break;
    case State::Idle:
      new_pattern = patterns_.idle;
      break;
    case State::ShorePower:
      new_pattern = patterns_.shore_power;
      break;
    case State::Charging:
      new_pattern = patterns_.manual_charging;
      break;
  }

  if (current_pattern_count_ >= new_pattern.size())
  {
    current_pattern_count_ = 0;
  }

  memcpy(&current_pattern_, &new_pattern[current_pattern_count_], sizeof(current_pattern_));

  old_state_ = state_;
  current_pattern_count_++;
}

}  // namespace dingo_base
