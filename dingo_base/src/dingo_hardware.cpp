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

#include <boost/assign.hpp>
#include <boost/asio.hpp> 
#include <boost/thread.hpp> 
#include <vector>

#include "puma_motor_driver/driver.hpp"
#include "puma_motor_driver/multi_driver_node.hpp"
#include "puma_motor_driver/socketcan_gateway.hpp"
#include "puma_motor_msgs/msg/multi_feedback.hpp"


#include "dingo_base/dingo_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dingo_base
{

hardware_interface::return_type DingoHardware::configure(
	const hardware_interface::HardwareInfo & info)
{
	active_ = false;

	if (configure_default(info) != hardware_interface::return_type::OK)
	{
		return hardware_interface::return_type::ERROR;
	}

	// Create gateway for the motor drivers
	std::string canbus_dev;
	canbus_dev = info_.hardware_parameters["canbus_dev"];
	gateway_ = std::shared_ptr<puma_motor_driver::SocketCANGateway>(new puma_motor_driver::SocketCANGateway(canbus_dev));

	// Read all the parameters for the robot
	gear_ratio_ = stod(info_.hardware_parameters["gear_ratio"]);
	encoder_cpr_ = stoi(info_.hardware_parameters["encoder_cpr"]);
	flip_motor_direction_ = stoi(info_.hardware_parameters["flip_motor_direction"]);
	gain_p_ = stod(info_.hardware_parameters["gains_p"]);
	gain_i_ = stod(info_.hardware_parameters["gains_i"]);
	gain_d_ = stod(info_.hardware_parameters["gains_d"]);

	for (const hardware_interface::ComponentInfo & joint : info_.joints)
	{
		// DiffBotSystem has exactly two states and one command interface on each joint
		if (joint.command_interfaces.size() != 1)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("controller_manager"),
				"Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
				joint.command_interfaces.size());
			return hardware_interface::return_type::ERROR;
		}

		if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("controller_manager"),
				"Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
				joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
			return hardware_interface::return_type::ERROR;
		}

		if (joint.state_interfaces.size() != 2)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("controller_manager"),
				"Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
				joint.state_interfaces.size());
			return hardware_interface::return_type::ERROR;
		}

		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("controller_manager"),
				"Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
				joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
				hardware_interface::HW_IF_POSITION);
			return hardware_interface::return_type::ERROR;
		}

		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("controller_manager"),
				"Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
				joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
			return hardware_interface::return_type::ERROR;
		}
	}

	std::vector<uint8_t> joint_can_ids;
	std::vector<float> joint_directions;
	if(info_.joints.size() == 2)
	{
		joint_can_ids.assign({2, 3});                       // NOLINT(whitespace/braces)
		joint_directions.assign({1, -1});                   // NOLINT(whitespace/braces)
	}
	else if(info_.joints.size() == 4)
	{
		joint_can_ids.assign({2, 3, 4, 5});                           // NOLINT(whitespace/braces)
		joint_directions.assign({1, -1, 1, -1});                      // NOLINT(whitespace/braces)
	}
	else
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("controller_manager"),
			"Unknown number of joints. Expected 2 or 4, got %d",
			info_.joints.size());
	}

	// Flip the motor direction if needed
	if (flip_motor_direction_)
	{
		for (std::size_t i = 0; i < joint_directions.size(); i++)
		{
			joint_directions[i] *= -1;
		}
	}

	for (uint8_t i = 0; i < info_.joints.size(); i++)
	{
		puma_motor_driver::Driver driver(gateway_, joint_can_ids[i], info_.joints[i].name);
		driver.clearMsgCache();
		driver.setEncoderCPR(encoder_cpr_);
		driver.setGearRatio(gear_ratio_ * joint_directions[i]);
		driver.setMode(puma_motor_msgs::msg::Status::MODE_SPEED, gain_p_, gain_i_, gain_d_);
		drivers_.push_back(driver);
	}

	multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode("multi_driver_node", drivers_));

	for (auto& driver : drivers_)
	{
		driver.configureParams();
	}

	std::thread can_read_thread(&DingoHardware::canReadThread, this);

	status_ = hardware_interface::status::CONFIGURED;
	return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DingoHardware::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	for (auto i = 0u; i < info_.joints.size(); i++)
	{
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].position));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity));
	}

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DingoHardware::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for (auto i = 0u; i < info_.joints.size(); i++)
	{
		command_interfaces.emplace_back(hardware_interface::CommandInterface(
			info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity_command));
	}

	return command_interfaces;
}

hardware_interface::return_type DingoHardware::start()
{
	RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "Starting ...please wait...");
	while (!connectIfNotConnected())
	{
		rclcpp::Rate(1).sleep();
	}

	this->verify();

	status_ = hardware_interface::status::STARTED;
	RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "System Successfully started!");
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type DingoHardware::stop()
{
	RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "Stopping ...please wait...");

	status_ = hardware_interface::status::STOPPED;

	RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "System successfully stopped!");

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type DingoHardware::read()
{
	if (this->isActive())
	{
		this->powerHasNotReset();
		this->updateJointsFromHardware();
	}
	else
	{
		for (auto& driver : drivers_)
		{
			driver.configureParams();
		}
	}

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type DingoHardware::write()
{
	if (this->isActive())
	{
		this->command();
		this->requestData();
	}
	else
	{
		this->verify();
	}
	return hardware_interface::return_type::OK;
}

bool DingoHardware::connectIfNotConnected()
{
	if (!gateway_->isConnected())
	{
		if (!gateway_->connect())
		{
			RCLCPP_ERROR(
				rclcpp::get_logger("controller_manager"),
				"Error connecting to motor driver gateway. Retrying in 1 second.");
			return false;
		}
		else
		{
			RCLCPP_INFO(
				rclcpp::get_logger("controller_manager"),
				"Connection to motor driver gateway successful.");
		}
	}
	return true;
}

void DingoHardware::verify()
{
	for (auto& driver : drivers_)
	{
		driver.verifyParams();
	}
}

bool DingoHardware::areAllDriversActive()
{
	for (auto& driver : drivers_)
	{
		if (!driver.isConfigured())
		{
			return false;
		}
	}
	return true;
}

bool DingoHardware::isActive()
{
	if (!active_ && this->areAllDriversActive())
	{
		active_ = true;
		multi_driver_node_->activePublishers(active_);
		RCLCPP_INFO(
			rclcpp::get_logger("controller_manager"),
			"Dingo Hardware Active");
	}
	else if (!this->areAllDriversActive() && active_)
	{
		active_ = false;
	}

	if (!active_)
	{
		RCLCPP_WARN(
			rclcpp::get_logger("controller_manager"),
			"Dingo Hardware Inactive");
	}

	return active_;
}

void DingoHardware::powerHasNotReset()
{
	// Checks to see if power flag has been reset for each driver
	for (auto& driver : drivers_)
	{
		if (driver.lastPower() != 0)
		{
			active_ = false;
			RCLCPP_WARN(
				rclcpp::get_logger("controller_manager"),
				"There was a power reset on Dev: %d, will reconfigure all drivers.", driver.deviceNumber());
			multi_driver_node_->activePublishers(active_);
			for (auto& driver : drivers_)
			{
				driver.resetConfiguration();
			}
		}
	}
}

void DingoHardware::command()
{
	uint8_t i = 0;
	for (auto& driver : drivers_)
	{
		driver.commandSpeed(joints_[i].velocity_command);
		i++;
	}
}

void DingoHardware::requestData()
{
	for (auto& driver : drivers_)
	{
		driver.requestFeedbackPowerState();
	}
}

void DingoHardware::updateJointsFromHardware()
{
	uint8_t index = 0;
	for (auto& driver : drivers_)
	{
		Joint* f = &joints_[index];
		f->effort = driver.lastCurrent();
		f->position = driver.lastPosition();
		f->velocity = driver.lastSpeed();
		index++;
	}
}

void DingoHardware::canReadThread()
{
	rclcpp::Rate rate(200);
	while (rclcpp::ok())
  {
    this->canRead();
    rate.sleep();
  }
}

void DingoHardware::canRead()
{
	puma_motor_driver::Message recv_msg;
	while (gateway_->recv(&recv_msg))
	{
		for (auto& driver : drivers_)
		{
			driver.processMessage(recv_msg);
		}
	}
}

}  // namespace dingo_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	dingo_base::DingoHardware, hardware_interface::SystemInterface)