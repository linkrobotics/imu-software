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

#include <Eigen/Dense>

using namespace std::chrono_literals;

/* linkrobotics imu driver */

class lrImuDriver : public rclcpp::Node
{
  public:
	lrImuDriver()
	: Node("lr_imu")
	{	
		RCLCPP_INFO(this->get_logger(), "Link Robotics Imu ROS2 Mag. Calibration Node v1.0 started");
		
		this->declare_parameter("port_name", "/dev/ttyUSB0");
		std::string portname = this->get_parameter("port_name").as_string();
		
		this->declare_parameter("baud_rate", 921600);
		//uint32_t currentBaudRate = std::atoi(this->get_parameter("baud_rate").as_string().c_str());
		uint32_t currentBaudRate = this->get_parameter("baud_rate").as_int();
	
		RCLCPP_INFO(this->get_logger(), "Port = %s Baud = %d", portname.c_str(), currentBaudRate);

		this->declare_parameter("save_calibration_to_flash", false);
		bool saveCalibrationToFlash = this->get_parameter("save_calibration_to_flash").as_bool();
		
		this->declare_parameter("calibration_time", 60);
		int calibrationTime = this->get_parameter("calibration_time").as_int();
		
		RCLCPP_INFO(this->get_logger(), "Calibration time = %d s", calibrationTime);
		RCLCPP_INFO(this->get_logger(), "Save to flash = %d", saveCalibrationToFlash);
		
		// create timer
		//timer_ = this->create_wall_timer(
		//	0.1ms, std::bind(&lrImuDriver::timer_callback, this));
		
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
		
		{
			RCLCPP_INFO(this->get_logger(), "In flash magnetometer calibration ...");
			
			RCLCPP_INFO(this->get_logger(), " Erasing flash memory. Please wait ...");
			if (imuPtr->startCalibration() == true) {
				RCLCPP_INFO(this->get_logger(), " Measurement started");
			}
			
			std::this_thread::sleep_for(std::chrono::seconds(calibrationTime));
			
			if (imuPtr->stopCalibration() == true) {
				RCLCPP_INFO(this->get_logger(), " Measurement stopped");
			}
			
			uint32_t size = imuPtr->getCalibrationDataSize();
			RCLCPP_INFO(this->get_logger(), " Calibration data size = %d", size);
			
			RCLCPP_INFO(this->get_logger(), " Reading flash memory ...");
			
			uint32_t blockSize = 128;
			std::vector<float> measurements;
			for (uint32_t addr = 0; addr < size; addr += blockSize) {
				RCLCPP_INFO(this->get_logger(), "  Reading block %x", addr);
				
				uint32_t endAddr = 0x0;
				if (addr + blockSize > size) {
					endAddr = size - addr - 32;
				}
				else {
					endAddr = addr + blockSize;
				}
				
				std::vector<float> arr = imuPtr->getCalibrationData(addr, endAddr);
				measurements.insert(measurements.end(), arr.begin(), arr.end());
				RCLCPP_INFO(this->get_logger(), "  Done");
			}
			RCLCPP_INFO(this->get_logger(), " Finished: %d measurements read", measurements.size());
			
			std::cout << "measurements: ";
			for (auto& m : measurements) {
				std::cout << m << " ";
			}
			std::cout << std::endl;
			
			samples_.clear();
			for (int i = 0; i < measurements.size()/3; i++) {
			    Eigen::Vector3f v(measurements[3*i + 0], measurements[3*i + 1], measurements[3*i + 2]);
			    samples_.push_back({v.x(), v.y(), v.z()});
			}
		}
		////////////////////////////////////////
			
		RCLCPP_INFO(this->get_logger(), "Calibrating ...");
		
		calculate();
		
		Eigen::Vector3f biasToSend = -bias_;
		
		std::cout << "b = " << biasToSend.transpose() << std::endl;
		std::cout << "L = " << L_ << std::endl;
		
		RCLCPP_INFO(this->get_logger(), " Done");
		
		
		RCLCPP_INFO(this->get_logger(), " Sending to device ...");
		
		bool ok = imuPtr->setMagneticFieldCalibrationData(biasToSend.data(), L_.data());
		
		if (ok == true) {
			RCLCPP_INFO(this->get_logger(), " Sent to device");
			
			if (saveCalibrationToFlash == true) {
				RCLCPP_INFO(this->get_logger(), " Saving to flash ...");
				if (imuPtr->saveSettings() == true) {
					RCLCPP_INFO(this->get_logger(), "  Done");
				}
			}
		}
		    
		
	}

  private:
	std::shared_ptr<imu> imuPtr;
  	std::shared_ptr<std::thread> thPtr;
  	
	
	// rclcpp::TimerBase::SharedPtr timer_;
	
	//
	std::vector<std::array<float,3>> samples_;

	Eigen::Vector3f bias_  = Eigen::Vector3f::Zero();
	Eigen::Vector3f scale_ = Eigen::Vector3f::Ones();
	Eigen::Matrix3f A_     = Eigen::Matrix3f::Identity(); // simetrik PD
	Eigen::Matrix3f L_     = Eigen::Matrix3f::Identity(); // A = L^T L
	
	
	Eigen::Matrix3f fitEllipsoidGN(
	    const std::vector<std::array<float,3>>& samples,
	    Eigen::Vector3f& bias)
	{
	    const int N = (int)samples.size();

	    // ----- initialize -----
	    float minX=1e9f, minY=1e9f, minZ=1e9f;
	    float maxX=-1e9f, maxY=-1e9f, maxZ=-1e9f;
	    for (auto &s : samples) {
		minX = std::min(minX, s[0]); maxX = std::max(maxX, s[0]);
		minY = std::min(minY, s[1]); maxY = std::max(maxY, s[1]);
		minZ = std::min(minZ, s[2]); maxZ = std::max(maxZ, s[2]);
	    }
	    bias = Eigen::Vector3f((maxX+minX)*0.5f,
		                   (maxY+minY)*0.5f,
		                   (maxZ+minZ)*0.5f );

	    float sx = (maxX-minX)*0.5f;
	    float sy = (maxY-minY)*0.5f;
	    float sz = (maxZ-minZ)*0.5f;
	    sx = std::max(sx, 1e-3f);
	    sy = std::max(sy, 1e-3f);
	    sz = std::max(sz, 1e-3f);

	    // A (diagonal): 1/s^2
	    Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
	    A(0,0) = 1.0f/(sx*sx);
	    A(1,1) = 1.0f/(sy*sy);
	    A(2,2) = 1.0f/(sz*sz);

	    // [a00, a01, a02, a11, a12, a22, b0, b1, b2]
	    const int P = 9;
	    const int maxIter = 15;
	    float damping = 1e-6f;

	    for (int it=0; it<maxIter; ++it) {
		Eigen::MatrixXf J(N, P);
		Eigen::VectorXf r(N);

		float a00=A(0,0), a01=A(0,1), a02=A(0,2),
		    a11=A(1,1), a12=A(1,2), a22=A(2,2);

		for (int i=0; i<N; ++i) {
		    Eigen::Vector3f m(samples[i][0], samples[i][1], samples[i][2]);
		    Eigen::Vector3f d = m - bias;

		    float dx=d(0), dy=d(1), dz=d(2);

		    // residual
		    float ri = a00*dx*dx + a11*dy*dy + a22*dz*dz
		               + 2.0f*a01*dx*dy + 2.0f*a02*dx*dz + 2.0f*a12*dy*dz - 1.0f;
		    r(i) = ri;

		    // Jacobian wrt [a00, a01, a02, a11, a12, a22]
		    J(i,0) = dx*dx;
		    J(i,1) = 2.0f*dx*dy;
		    J(i,2) = 2.0f*dx*dz;
		    J(i,3) = dy*dy;
		    J(i,4) = 2.0f*dy*dz;
		    J(i,5) = dz*dz;

		    // Jacobian wrt bias
		    Eigen::Vector3f grad_b = -2.0f * (A * d);
		    J(i,6) = grad_b(0);
		    J(i,7) = grad_b(1);
		    J(i,8) = grad_b(2);
		}

		Eigen::MatrixXf H = J.transpose() * J;
		H += damping * Eigen::MatrixXf::Identity(P, P);
		Eigen::VectorXf g = J.transpose() * r;

		Eigen::VectorXf delta = -H.ldlt().solve(g);

		a00 += delta(0);
		a01 += delta(1);
		a02 += delta(2);
		a11 += delta(3);
		a12 += delta(4);
		a22 += delta(5);
		bias(0) += delta(6);
		bias(1) += delta(7);
		bias(2) += delta(8);
		A << a00, a01, a02,
		    a01, a11, a12,
		    a02, a12, a22;

		A = 0.5f * (A + A.transpose());
		if (delta.norm() < 1e-6f) break;
   	 }

	    Eigen::LLT<Eigen::Matrix3f> llt(A);
	    if (llt.info() != Eigen::Success) {
		A += 1e-6f * Eigen::Matrix3f::Identity();
	    }
	    return A;
	}
	
	
	void computeQualityMetrics(
	    const std::vector<std::array<float,3>>& samples,
	    const Eigen::Vector3f& bias,
	    const Eigen::Matrix3f& L,
	    float& avgNorm, float& stdNorm)
	{
	    const int N = static_cast<int>(samples.size());
	    if (N == 0) {
		avgNorm = stdNorm = NAN;
		return;
	    }

	    double sum = 0.0, sum2 = 0.0;
	    for (auto &s : samples) {
		Eigen::Vector3f m(s[0], s[1], s[2]);
		Eigen::Vector3f p = L * (m - bias);  // corrected point
		double n = p.norm();
		sum  += n;
		sum2 += n * n;
	    }
	    double mean = sum / N;
	    double var  = std::max(0.0, (sum2 / N) - (mean * mean));
	    avgNorm = static_cast<float>(mean);
	    stdNorm = static_cast<float>(std::sqrt(var));

	    // GUI update
	    RCLCPP_INFO(this->get_logger(), "Avg. norm = %f", avgNorm);
	    RCLCPP_INFO(this->get_logger(), "Std. norm = %f", stdNorm);
	    RCLCPP_INFO(this->get_logger(), "Samples = %d", N);
	   
	    std::string quality = qualityLabel(stdNorm);
	    RCLCPP_INFO(this->get_logger(), "%s", quality.c_str());
	}
	
	std::string qualityLabel(float stdNorm)
	{
	    // heuristik
	    if (stdNorm < 0.03f) return ("Good");
	    if (stdNorm < 0.08f) return ("Acceptable");
	    return ("Bad");
	}
	
	void calculate() {
	    A_ = fitEllipsoidGN(samples_, bias_);
	    Eigen::Matrix3f Asym = 0.5f * (A_ + A_.transpose());
	    Eigen::LLT<Eigen::Matrix3f> llt(Asym);
	    if (llt.info() != Eigen::Success) {
		Asym += 1e-6f * Eigen::Matrix3f::Identity();
		llt.compute(Asym);
	    }
	    L_ = llt.matrixU();
	    scale_ = Eigen::Vector3f(1.0/L_(0,0),
		                     1.0/L_(1,1),
		                     1.0/L_(2,2));


	    float avgNorm=0.f, stdNorm=0.f;
	    computeQualityMetrics(samples_, bias_, L_, avgNorm, stdNorm);
	    
	    //
	    
	}
	
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<lrImuDriver>());
	rclcpp::shutdown();
	
	return 0;
}

