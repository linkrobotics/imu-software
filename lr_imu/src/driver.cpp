/**
  ******************************************************************************
  * @file           : driver.cpp
  * @brief          : IMU ROS2 driver source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Link Robotics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <exception>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <boost/shared_ptr.hpp>

#include <imu.h>


using namespace std::chrono_literals;

/* linkrobotics imu driver */

class lrImuDriver : public rclcpp::Node
{
  public:
	lrImuDriver()
	: Node("lr_imu")
	{	
		RCLCPP_INFO(this->get_logger(), "Link Robotics Imu ROS2 Driver v1.0 started");
		
		this->declare_parameter("port_name", "/dev/ttyUSB0");
		std::string portname = this->get_parameter("port_name").as_string();
		
		this->declare_parameter("baud_rate", 921600);
		//uint32_t currentBaudRate = std::atoi(this->get_parameter("baud_rate").as_string().c_str());
		uint32_t currentBaudRate = this->get_parameter("baud_rate").as_int();
	
		RCLCPP_INFO(this->get_logger(), "Port = %s Baud = %d", portname.c_str(), currentBaudRate);

		
		// create timer
		timer_ = this->create_wall_timer(
			0.1ms, std::bind(&lrImuDriver::timer_callback, this));
		
		////////////////////////////////////////
		// create imu object
		imuPtr.reset(new imu);
		// connect to serial port
		imuPtr->connect(portname, currentBaudRate, 8, 0, 1);
		// start thread
		imuPtr->start();
		
		// spin for thread
		thPtr.reset(new std::thread([&] { 
			imuPtr->spin(); 
			} ));
		////////////////////////////////////////
		
		
		////////////////////////////////////////
		// device config
		if (true) {
			RCLCPP_INFO(this->get_logger(), "Loading settings ...");
			
			
			if (imuPtr->loadSettings() == true) {
				uint32_t baudRate = imuPtr->getBaudRate();
				if (baudRate > 0) {
					RCLCPP_INFO(this->get_logger(), " Baud rate is  %u", baudRate);
					
				}
				
				uint16_t dataRate = imuPtr->getDataRate();
				if (dataRate > 0) {
					RCLCPP_INFO(this->get_logger(), " Data rate is %u", dataRate);
					
				}
				
				uint16_t outputSetting = imuPtr->getOutputs();
				if (outputSetting != 0) {
					RCLCPP_INFO(this->get_logger(), " Output setting is %u", outputSetting);
				}
			}
		}
		
				
		RCLCPP_INFO(this->get_logger(), "Publishing /lr/imu/data ...");
		publisherImu_ = this->create_publisher<sensor_msgs::msg::Imu>("/lr/imu/data", 10);
		
		RCLCPP_INFO(this->get_logger(), "Publishing /lr/imu/magnetic_field ...");
		publisherMagField_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/lr/imu/magnetic_field", 10);
		
		RCLCPP_INFO(this->get_logger(), "Publishing /lr/imu/euler_angles ...");
		publisherEuler_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/lr/imu/euler_angles", 10);
	
		
		RCLCPP_INFO(this->get_logger(), "Publishing ...");
	}

  private:
	std::shared_ptr<imu> imuPtr;
  	std::shared_ptr<std::thread> thPtr;
  	
	std::vector<float> q, euler, accels, rates, mag, cov;
	float altitude;
  
  
	void timer_callback()
	{		
		// get quaternion data
		if ((q = imuPtr->getQuaternion()).size() > 0) {
			
			//RCLCPP_INFO(this->get_logger(), "q = %f %f %f %f", q[0], q[1], q[2], q[3]);
			
			auto imuMsg = sensor_msgs::msg::Imu();
				
			imuMsg.header.stamp = now();
			imuMsg.header.frame_id = "lr_imu_link";
			
			imuMsg.orientation.x = q[0];
			imuMsg.orientation.y = q[1];
			imuMsg.orientation.z = q[2];
			imuMsg.orientation.w = q[3];
			
			if ((rates = imuPtr->getRates()).size() > 0) {
				//RCLCPP_INFO(this->get_logger(), "w = %f %f %f", rates[0], rates[1], rates[2]);
			
				imuMsg.angular_velocity.x = rates[0];
				imuMsg.angular_velocity.y = rates[1];
				imuMsg.angular_velocity.z = rates[2];
			}
			
			if ((accels = imuPtr->getAccelerations()).size() > 0) {
				//RCLCPP_INFO(this->get_logger(), "a = %f %f %f", accels[0], accels[1], accels[2]);
				
				imuMsg.linear_acceleration.x = accels[0];
				imuMsg.linear_acceleration.y = accels[1];
				imuMsg.linear_acceleration.z = accels[2];
			}
			
			if ((cov = imuPtr->getCovariances()).size() > 0) {
				imuMsg.orientation_covariance[0 + 0 * 3] = cov[0];
				imuMsg.orientation_covariance[1 + 1 * 3] = cov[1];
				imuMsg.orientation_covariance[2 + 2 * 3] = cov[2];
			}
			
			publisherImu_->publish(imuMsg);
		}
		
		// get magnetic field data
		if ((mag = imuPtr->getMagneticField()).size() > 0) {
			auto magMsg = sensor_msgs::msg::MagneticField();

			magMsg.header.stamp = now();
			magMsg.header.frame_id = "lr_imu_link";

			magMsg.magnetic_field.x = mag[0];
			magMsg.magnetic_field.y = mag[1];
			magMsg.magnetic_field.z = mag[2];

			publisherMagField_->publish(magMsg);
		}
		
		
		// get euler angles
		if ((euler = imuPtr->getEuler()).size() > 0) {
			auto eulerMsg = geometry_msgs::msg::Vector3Stamped();
			
			eulerMsg.header.stamp = now();
			eulerMsg.header.frame_id = "lr_imu_link";

			eulerMsg.vector.x = euler[0];
			eulerMsg.vector.y = euler[1];
			eulerMsg.vector.z = euler[2];

			publisherEuler_->publish(eulerMsg);
		
			//RCLCPP_INFO(this->get_logger(), "euler (r-p-y) = %f %f %f", euler[0], euler[1], euler[2]);
		}
	}
	
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisherImu_;
	rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisherMagField_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisherEuler_;


};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<lrImuDriver>());
	rclcpp::shutdown();
	
	return 0;
}

