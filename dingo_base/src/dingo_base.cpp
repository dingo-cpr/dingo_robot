/**
 *  \file       dingo_base.cpp
 *  \brief      Main entry point for dingo base
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

#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include "controller_manager/controller_manager.h"
#include "dingo_base/dingo_diagnostic_updater.h"
#include "dingo_base/dingo_hardware.h"
#include "dingo_base/dingo_cooling.h"
#include "dingo_base/dingo_lighting.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "ros/ros.h"
#include "rosserial_server/udp_socket_session.h"

using boost::asio::ip::udp;
using boost::asio::ip::address;

typedef boost::chrono::steady_clock time_source;

/** This is the main thread for monitoring and updating the robot.
 *  Assumes a separate thread is used for this function as it never returns.
 *  @param[in] rate Controls the rate at which each loop iteration is run
 *  @param[in] robot Handle to the hardware for the robot
 *  @param[in] cm Handle to the controller manager
 */
void controlThread(ros::Rate rate, dingo_base::DingoHardware* robot, controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    if (robot->isActive())
    {
      robot->powerHasNotReset();
      robot->updateJointsFromHardware();
    }
    else
    {
      robot->configure();
    }

    robot->canSend();

    cm->update(ros::Time::now(), elapsed, robot->inReset());

    if (robot->isActive())
    {
      robot->command();
      robot->requestData();
    }
    else
    {
      robot->verify();
    }

    robot->canSend();
    rate.sleep();
  }
}

/** Perform reads on CAN bus. Assumes a separate thread is used for this
 *  function as it never returns.
 *  @param[in] rate Controls the rate at which CAN bus is read
 *  @param[in] robot Handle to the hardware for the robot
 */
void canReadThread(ros::Rate rate, dingo_base::DingoHardware* robot)
{
  while (1)
  {
    robot->canRead();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "dingo_node");
  ros::NodeHandle nh, pnh("~");

  // Create the socket rosserial server in a background ASIO event loop.
  boost::asio::io_service io_service;
  rosserial_server::UdpSocketSession* socket;
  boost::thread socket_thread;

  bool use_mcu = true;
  pnh.param<bool>("use_mcu", use_mcu, true);

  if (use_mcu)
  {
      socket = new rosserial_server::UdpSocketSession(io_service,
          udp::endpoint(address::from_string("192.168.131.1"), 11411),
          udp::endpoint(address::from_string("192.168.131.2"), 11411));
      socket_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
  }

  std::string canbus_dev;
  pnh.param<std::string>("canbus_dev", canbus_dev, "can0");
  puma_motor_driver::SocketCANGateway gateway(canbus_dev);

  // TODO(tbaltovski): how to choose between differential and mechanum
  dingo_base::DingoHardware dingo(dingo_base::DingoHardware::DingoType::DINGO_D, nh, pnh, gateway);

  // Configure the CAN connection
  dingo.init();
  // Create a thread to start reading can messages.
  boost::thread canReadT(&canReadThread, ros::Rate(200), &dingo);

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&dingo, controller_nh);
  boost::thread controlT(&controlThread, ros::Rate(25), &dingo, &cm);

  // Lighting control.
  dingo_base::DingoLighting* lighting;
  if (use_mcu)
  {
    lighting = new dingo_base::DingoLighting(&nh);
  }

  // Create diagnostic updater, to update itself on the ROS thread.
  dingo_base::DingoDiagnosticUpdater dingo_diagnostic_updater;
  puma_motor_driver::PumaMotorDriverDiagnosticUpdater puma_motor_driver_diagnostic_updater;

  // Cooling control for the fans.
  dingo_base::DingoCooling* cooling;
  if (use_mcu)
  {
      cooling = new dingo_base::DingoCooling(&nh);
  }

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}
