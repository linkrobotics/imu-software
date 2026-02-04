/**
  ******************************************************************************
  * @file           : imu.cpp
  * @brief          : IMU driver source file
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
  
#include "imu.h"

imu::imu() {
	serialPtr.reset(new boost::asio::serial_port(io));
	
	transmitBuffer = new uint8_t[maxBufferSize];
	transmitPayload = new uint8_t[maxPayloadSize];
	
	calibrationData = new float[maxCalibrationDataSize];
}

imu::~imu() {
	delete [] transmitPayload;
	delete [] transmitBuffer;
	delete [] calibrationData;
}

bool imu::processPacket(const unsigned char* receiveBuffer, const int n) {
	bool checksumOk = false;
	
	uint16_t packetSize = uint16_t(receiveBuffer[2]) | (uint16_t(receiveBuffer[3]) << 8);
	
	if (packetSize <= n) { //
		uint32_t checksum = 0;

		for (int i = 4; i < packetSize - 2; i++) {
		    checksum += uint32_t(receiveBuffer[i]);
		}
		uint16_t checksum16 = uint16_t(checksum & 0xffff);

		uint16_t packetChecksum = uint16_t(receiveBuffer[packetSize - 2]) | (uint16_t(receiveBuffer[packetSize - 1]) << 8);

		if (packetChecksum == checksum16) {
			checksumOk = true;
		}
	}
	
	if (checksumOk == true) {
		const int start = 4;
		
		int i = start;
		
		int payloadEnd = n - 2;
		

		
		while (i < payloadEnd) {
			
			if (receiveBuffer[1] == 'D') {
				if (receiveBuffer[i] == payloadIdQuaternion) {
					i++;
					bytesToFloat(&receiveBuffer[i], &q[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &q[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &q[2]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &q[3]);
					i += sizeof(float);
										
					outputFlags[payloadIdQuaternion] = true;
					
				}
				else if (receiveBuffer[i] == payloadIdCovariance) {
					i++;
					bytesToFloat(&receiveBuffer[i], &cov[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &cov[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &cov[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdCovariance] = true;
				}
				else if (receiveBuffer[i] == payloadIdMagneticField) {
					i++;
					bytesToFloat(&receiveBuffer[i], &mag[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &mag[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &mag[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdMagneticField] = true;
				}
				else if (receiveBuffer[i] == payloadIdMagneticFieldRaw) {
					i++;
					bytesToFloat(&receiveBuffer[i], &magRaw[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &magRaw[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &magRaw[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdMagneticFieldRaw] = true;
				}
				else if (receiveBuffer[i] == payloadIdLinearAcceleration) {
					i++;
					bytesToFloat(&receiveBuffer[i], &accels[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &accels[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &accels[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdLinearAcceleration] = true;
				}
				else if (receiveBuffer[i] == payloadIdAngularRate) {
					i++;
					bytesToFloat(&receiveBuffer[i], &rates[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &rates[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &rates[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdAngularRate] = true;
				}
				else if (receiveBuffer[i] == payloadIdAltitude) {
					i++;
					bytesToFloat(&receiveBuffer[i], &altitude);
					i += sizeof(float);
					
					
					outputFlags[payloadIdAltitude] = true;
				}
				else if (receiveBuffer[i] == payloadIdEulerAngles) {
					i++;
					bytesToFloat(&receiveBuffer[i], &euler[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &euler[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &euler[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdEulerAngles] = true;
				}
				else if (receiveBuffer[i] == payloadIdDebugRaw) {
					i++;
					bytesToFloat(&receiveBuffer[i], &accelsRaw[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &accelsRaw[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &accelsRaw[2]);
					i += sizeof(float);
					
					bytesToFloat(&receiveBuffer[i], &ratesRaw[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &ratesRaw[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &ratesRaw[2]);
					i += sizeof(float);
					
					bytesToFloat(&receiveBuffer[i], &magRaw[0]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &magRaw[1]);
					i += sizeof(float);
					bytesToFloat(&receiveBuffer[i], &magRaw[2]);
					i += sizeof(float);
					
					outputFlags[payloadIdDebugRaw] = true;
				}
				else {
					i++;	
				}
			}
			else if (receiveBuffer[1] == 'G') {
				
			
				// settings
				if (receiveBuffer[i] == payloadIdBaudRate) {
					i++;
					bytesToUInt32(&receiveBuffer[i], &baudRate);
					i += sizeof(uint32_t);
					
					outputFlags[payloadIdBaudRate] = true;
					
				}
				else if (receiveBuffer[i] == payloadIdOutputDataRate) {
					i++;
					bytesToUInt16(&receiveBuffer[i], &dataRate);
					i += sizeof(uint16_t);
					
					outputFlags[payloadIdOutputDataRate] = true;
					
				}
				else if (receiveBuffer[i] == payloadIdOutputSetting) {
					i++;
					bytesToUInt16(&receiveBuffer[i], &outputSetting);
					i += sizeof(uint16_t);
					
					outputFlags[payloadIdOutputSetting] = true;
				}
				else if (receiveBuffer[i] == payloadIdSaveSettings) {
					i++;
					
					if (receiveBuffer[i++] == 1) {
						outputFlags[payloadIdSaveSettings] = true;
					}
				}
				else if (receiveBuffer[i] == payloadIdRevertToFactory) {
					i++;
					
					if (receiveBuffer[i++] == 1) {
						outputFlags[payloadIdRevertToFactory] = true;
					}
				}
				else if (receiveBuffer[i] == payloadIdFirmwareInfo) {
				    i++;
				    bytesToUInt16(&receiveBuffer[i], &firmwareVersion);
				    i += sizeof(uint16_t);
				    bytesToUInt16(&receiveBuffer[i], &hardwareVersion);
				    i += sizeof(uint16_t);


				    outputFlags[payloadIdFirmwareInfo] = true;
				}
				else if (receiveBuffer[i] == payloadIdSerialNumber) {
				    i++;
				    bytesToUInt32(&receiveBuffer[i], &serialNumber);
				    i += sizeof(uint32_t);

				    outputFlags[payloadIdSerialNumber] = true;
				}
				else if (receiveBuffer[i] == payloadIdMagneticFieldCalibrationData) {
				    i++;
				    bytesToFloat(&receiveBuffer[i], &bias_[0]);
				    i += sizeof(float);
				    bytesToFloat(&receiveBuffer[i], &bias_[1]);
				    i += sizeof(float);
				    bytesToFloat(&receiveBuffer[i], &bias_[2]);
				    i += sizeof(float);
				    
				    for (int ii = 0; ii < 3*3; ii++) {
				    	bytesToFloat(&receiveBuffer[i], &scale_[ii]);
				    	i += sizeof(float);
				    }
				    

				    outputFlags[payloadIdMagneticFieldCalibrationData] = true;
				}
				else if (receiveBuffer[i] == payloadIdStartCalibration) {
					i++;
				
					//std::cout << " payloadIdStartCalibration" << std::endl; 
					
					if (receiveBuffer[i++] == 1) {
						outputFlags[payloadIdStartCalibration] = true;
						
						
						
					}
				}
				else if (receiveBuffer[i] == payloadIdStopCalibration) {
					i++;
					
					
					if (receiveBuffer[i++] == 1) {
						outputFlags[payloadIdStopCalibration] = true;
						
					}
				}
				else if (receiveBuffer[i] == payloadIdGetCalibrationData) {
					
					
					i++;
					if ((payloadEnd - 1) + 1 <= maxCalibrationDataSize) {
						
						int n = 0;
						while (i < payloadEnd) {
							bytesToFloat(&receiveBuffer[i], &calibrationData[n++]);
							i += sizeof(float);
							bytesToFloat(&receiveBuffer[i], &calibrationData[n++]);
							i += sizeof(float);
							bytesToFloat(&receiveBuffer[i], &calibrationData[n++]);
							i += sizeof(float);
						}
						calibrationDataSize_ = n;
						outputFlags[payloadIdGetCalibrationData] = true;
					}
					
				}
				else if (receiveBuffer[i] == payloadIdGetCalibrationDataSize) {
					i++;
					
					bytesToUInt32(&receiveBuffer[i], &calibrationDataSize);
					
					outputFlags[payloadIdGetCalibrationDataSize] = true;
					
				}
				else if (receiveBuffer[i] == payloadIdZeroThreshold) {
					i++;
					
					bytesToFloat(&receiveBuffer[i], &zeroThreshold);
					i += sizeof(float);
					
					outputFlags[payloadIdZeroThreshold] = true;
				}
				else if (receiveBuffer[i] == payloadIdDebugConfig) {
					i++;
					
					i += 11 * sizeof(float);
					
					outputFlags[payloadIdDebugConfig] = true;
				}
				
				else {
					i++;	
				}
			}
			else {
				i++;
			}
		}
		
		return true;
	}
	else {
		
		return false;
	}
}

void imu::loop() {
	
	while (true) {
		
		if (connected == true) {
			
			
			
			////////////////////////////////
			unsigned char* buffer = new unsigned char[2048];
			int n = 0;
			//int packetSize = 0;
			
			// header
			waitForNBytes(1);
			n += readNBytes(1, buffer);
			
			if (buffer[0] == 'S') {
				// packet type
				waitForNBytes(1);
				n += readNBytes(1, &buffer[1]);
				
				if (buffer[1] == 'D' || buffer[1] == 'G') {
					// data
					waitForNBytes(2);
					n += readNBytes(2, &buffer[2]);
				
					unsigned short length = 0;
					bytesToUInt16(&buffer[2], &length);
					
					if (length < 256 && length >= 4) {
					
						waitForNBytes(length - 4);
						n += readNBytes(length - 4, &buffer[4]);
						
						mutex1.lock();
						if (processPacket(buffer, n) == false) {
							//std::cout << "checksum not ok" << std::endl;
						}
						
						mutex1.unlock();
						
					}
				}
			}
			
			delete [] buffer;
			
			////////////////////////////////
			
			
			
		}		
		
		
		boost::this_thread::sleep_for(boost::chrono::microseconds(10));
	}
}

void imu::start() {	
	threadPtr.reset(new boost::thread(&imu::loop, this));
}

void imu::stop() {	
	threadPtr->interrupt();
}


void imu::spin() {
	threadPtr->join();
}

bool imu::connect(const std::string port, const int baudRate, const int dataBits, const int parity, const int stopBits) {
	boost::lock_guard<boost::mutex> lock(mutex1);		
	
	
	if (connected == false) {
		
		portname = port;
		this->baudRate = baudRate;

		serialPtr->open(portname);
		
		/*if (parity == 0) {
			serialPtr->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		}
		else if (parity == 1) {
			serialPtr->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
		}
		else if (parity == 2) {
			serialPtr->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
		}
		
		if (stopBits == 1) {
			serialPtr->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		}
		if (stopBits == 2) {
			serialPtr->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
		}
		if (stopBits == 3) {
			serialPtr->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::onepointfive));
		}
		*/
		// TODO
		serialPtr->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
		serialPtr->set_option(boost::asio::serial_port_base::character_size(dataBits));
		
		
		
		connected = true;
	}
	
	return true;
}

bool imu::disconnect() {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == true) { 
		serialPtr->close();
		connected = false;
	}
	return true;		
}


bool imu::getQuaternion_(float* q_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdQuaternion];
	
	outputFlags[payloadIdQuaternion] = false;
	
	if (ready == true) {
		memcpy(q_, q, 4 * sizeof(float));
	}

	return ready;
}

bool imu::getEuler_(float* euler_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
		
	bool ready = outputFlags[payloadIdEulerAngles];
	
	outputFlags[payloadIdEulerAngles] = false;
	
	if (ready == true) {
		memcpy(euler_, euler, 3 * sizeof(float));
	}
	
	return ready;
}

bool imu::getAltitude_(float* altitude_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdAltitude];
	
	if (ready == true) {
		*altitude_ = altitude; 
	}
	outputFlags[payloadIdAltitude] = false;
	
	return ready;
}

bool imu::getAccelerations_(float* accels_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdLinearAcceleration];
	
	outputFlags[payloadIdLinearAcceleration] = false;
	if (ready == true) {
		memcpy(accels_, accels, 3 * sizeof(float));
	}
	return ready;
}

bool imu::getRates_(float* rates_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdAngularRate];
	
	outputFlags[payloadIdAngularRate] = false;
	if (ready == true) {
		memcpy(rates_, rates, 3 * sizeof(float));
	}
	return ready;

}

bool imu::getMagneticField_(float* mag_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdMagneticField];
	
	outputFlags[payloadIdMagneticField] = false;
	if (ready == true) {
		memcpy(mag_, mag, 3 * sizeof(float));
	}
	return ready;
}

bool imu::getMagneticFieldRaw_(float* magRaw_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdMagneticFieldRaw];
	
	outputFlags[payloadIdMagneticFieldRaw] = false;
	if (ready == true) {
		memcpy(magRaw_, magRaw, 3 * sizeof(float));
	}
	return ready;
}


bool imu::getDebugRaw_(float* accelsRaw_, float* ratesRaw_, float* magRaw_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdDebugRaw];
	
	outputFlags[payloadIdDebugRaw] = false;
	if (ready == true) {
		memcpy(accelsRaw_, accelsRaw, 3 * sizeof(float));
		memcpy(ratesRaw_, ratesRaw, 3 * sizeof(float));
		memcpy(magRaw_, magRaw, 3 * sizeof(float));
	}
	return ready;
}


bool imu::getCovariances_(float* cov_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdCovariance];
	
	outputFlags[payloadIdCovariance] = false;
	if (ready == true) {
		memcpy(cov_, cov, 3 * sizeof(float));
	}
	return ready;
}

bool imu::getDataRate_(uint16_t* dataRate_) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}

	bool ready = outputFlags[payloadIdOutputDataRate];
	
	outputFlags[payloadIdOutputDataRate] = false;
	if (ready == true) {
		*dataRate_ = dataRate;
	}
	return ready;
}

uint16_t imu::getDataRate() {
	if (connected == false) {
		return 0;
	}

	uint16_t dataRate = 0;
	if (getDataRate_(&dataRate) == true)  {
		return dataRate;
	}
	
	return 0;
}





bool imu::setDataRate_(const int dataRate) {
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadDataRate(transmitPayload, dataRate);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);

	mutex1.unlock();

	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdOutputDataRate) == true) {
			if (this->dataRate == dataRate) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
} 

bool imu::setDataRate(const int dataRate) {
	if (connected == false) {
		return false;
	}

	// set data rate
	while (true) {
		if (setDataRate_(dataRate) == true) {
			uint16_t dataRate = 0;
			
			if (getDataRate_(&dataRate) == true) {
				
				return true;
			}
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}


bool imu::getBaudRate_(uint32_t* baudRate_) {
	
	if (connected == false) {
		return false;
	}
	
	
	bool ready = outputFlags[payloadIdBaudRate];
	
	outputFlags[payloadIdBaudRate] = false;
	if (ready == true) {
		*baudRate_ = baudRate;
	}
	return ready;
}
uint32_t imu::getBaudRate() {
	if (connected == false) {
		return 0;
	}

	uint32_t baudRate = 0;
	if (getBaudRate_(&baudRate) == true) {
		return baudRate;
	}
	
	return 0;
}


bool imu::setBaudRate_(const uint32_t baudRate) {
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadBaudRate(transmitPayload, baudRate);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
	mutex1.unlock();
	
	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdBaudRate) == true) {
			if (this->baudRate == baudRate) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

bool imu::setBaudRate(const uint32_t newBaudRate) {
	
		
	if (connected == false) {
		return false;
	}
	
	// set baud rate
	int counter = 0;
	
	uint32_t currentBaudRate = portBaudRate;
	
	bool baudChangeSuccess = false;
	while (counter++ < 500) {
		if (baudChangeSuccess == false) {
			printf("set baud rate\n");
			float q[4];
			getQuaternion_(q);
			setBaudRate_(newBaudRate);
			
			printf("reconnect\n");
			disconnect();
			connect(portname, newBaudRate, 8, 0, 1);
			
			int counterQ = 0;
			while (true) {
				
				if (getQuaternion_(q) == true) {
					baudChangeSuccess = true;
					printf("success\n");
					break;
				}
				else {
					printf("change to new baud rate\n");
					setBaudRate_(newBaudRate);
					disconnect();
					connect(portname, newBaudRate, 8, 0, 1);
					
				}
				
				if (counterQ++ > 10) {
					printf("change to old baud rate\n");
				
					disconnect();
					connect(portname, currentBaudRate, 8, 0, 1);
					
					baudChangeSuccess = false;
					
					break;
				}
				boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
			}	
		}
		else {
			
			bool success = false;
			
			int counterSettings = 0;
			while (true) {
				//
				if (loadSettings() == true) {
					success = true;
					break;
				}
				
				if (counterSettings++ > 10) {
					break;
				}
				
				boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
			}
			
			if (success == true) {
				break;
			}
			
		}
		
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
		
	}
	
	

	uint32_t baudRate = getBaudRate();
	if (baudRate > 0) {
		return true;
	}
	
	return false;
}

bool imu::isSyncEnabled() {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	// TODO:
	
	return syncEnabled;
}
bool imu::setSyncEnabled(const bool syncEnabled) {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	// TODO:
	
	return true;
}

bool imu::setZeroThreshold_(float zeroThreshold) {
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadZeroThreshold(transmitPayload, zeroThreshold);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);

	mutex1.unlock();

	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdZeroThreshold) == true) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
} 

bool imu::setZeroThreshold(const float zeroThreshold) {
	if (connected == false) {
		return false;
	}

	// set data rate
	while (true) {
		if (setZeroThreshold_(zeroThreshold) == true) {
			return true;
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}

bool imu::setDebugConfig_(	float sigma_g,
				float sigma_wg,
				float alpha_g,
				float sigma_a,
				float sigma_ba,
				float alpha_a,
				float sigma_m,
				float sigma_h,
				float alpha_h,
				float sigma_hs,
				float sigma_z) {
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadDebugConfig(transmitPayload, 
							sigma_g,
							sigma_wg,
							alpha_g,
							sigma_a,
							sigma_ba,
							alpha_a,
							sigma_m,
							sigma_h,
							alpha_h,
							sigma_hs,
							sigma_z);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);

	mutex1.unlock();

	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdDebugConfig) == true) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
} 

bool imu::setDebugConfig(	const float sigma_g,
				const float sigma_wg,
				const float alpha_g,
				const float sigma_a,
				const float sigma_ba,
				const float alpha_a,
				const float sigma_m,
				const float sigma_h,
				const float alpha_h,
				const float sigma_hs,
				const float sigma_z) {
	if (connected == false) {
		return false;
	}

	// set data rate
	while (true) {
		if (setDebugConfig_(	sigma_g,
					sigma_wg,
					alpha_g,
					sigma_a,
					sigma_ba,
					alpha_a,
					sigma_m,
					sigma_h,
					alpha_h,
					sigma_hs,
					sigma_z) == true) {
			return true;
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}

bool imu::getOutputs_(uint16_t* outputBitMask_) {
	// get outputs bitmask from devices
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	if (connected == false) {
		return false;
	}
	
	bool ready = outputFlags[payloadIdOutputSetting];
	
	outputFlags[payloadIdOutputSetting] = false;
	if (ready == true) {
		*outputBitMask_ = outputSetting;
	}
	return ready;
}

uint16_t imu::getOutputs() {
	if (connected == false) {
		return 0;
	}
	
	uint16_t outputBitMask_ = 0;
	
	if (getOutputs_(&outputBitMask_) == true){
		return outputBitMask_;
	}	
	
	return 0;
}

bool imu::setOutputs_(uint16_t outputBitMask) {
	// set outputs on or off and send to device
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadOutputSetting(transmitPayload, outputBitMask);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
						
	mutex1.unlock();
						
	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdOutputSetting) == true) {
			if (this->outputSetting == outputBitMask) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

bool imu::setOutputs(uint16_t outputBitMask) {
	if (connected == false) {
		return false;
	}

	while (true) {
		if (setOutputs_(outputBitMask) == true) {
			uint16_t outputSetting = 0;
			
			if (getOutputs_(&outputSetting) == true) {
				
				return true;
				
			}
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}



bool imu::saveSettings_() {
	// save settings to device
	mutex1.lock();	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadSaveSettings(transmitPayload);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
	mutex1.unlock();
					
	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdSaveSettings) == true) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

bool imu::saveSettings() {			
		
	if (connected == false) {
		return false;
	}	
		
	while (true) {
		if (saveSettings_() == true) {
			return true;
		
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
	}
	
	return false;
}

bool imu::loadSettings_() {
	// load settings from device
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	// all settings here
	std::vector<uint32_t> payloadIds = std::vector<uint32_t>{ payloadIdBaudRate, payloadIdOutputDataRate, payloadIdOutputSetting };
	
	int nPayload = preparePayloadLoadSettings(transmitPayload, payloadIds);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
	
	mutex1.unlock();
	
	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponses(payloadIds) == true) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

bool imu::loadSettings() {
	if (connected == false) {
		return false;
	}

	resetSettingsFlags();
	
	int counter = 0;
	
	while (counter++ <= 100) {
		if (loadSettings_() == true) {	
			return true;
			break;
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}

void imu::resetSettingsFlags() {
	boost::lock_guard<boost::mutex> lock(mutex1);
	
	outputFlags[payloadIdBaudRate] = false;
	outputFlags[payloadIdOutputDataRate] = false;
	outputFlags[payloadIdOutputSetting] = false;
}

bool imu::revertToFactorySettings() {
	// factory settings
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	int nPayload = preparePayloadRevertToFactory(transmitPayload);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
		
		
	mutex1.unlock();
					
	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdRevertToFactory) == true) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}



bool imu::startCalibration() {
	
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	
	outputFlags[payloadIdStartCalibration] = false;
	
	int nPayload = preparePayloadStartCalibration(transmitPayload);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
						
	mutex1.unlock();
						
	int counter = 0;
	while (true) {
		//
		//std::cout << "sending " << std::endl;
		int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));
		if (n == nPacket) {
			if (waitForResponse(payloadIdStartCalibration, 10000) == true) {
				return true;
			}
			
		}
		
		if (counter++ > 10) {
			return false;
		}
		
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}
bool imu::stopCalibration() {
	mutex1.lock();
	if (connected == false) {
		return false;
	}
	
	outputFlags[payloadIdStopCalibration] = false;
	
	int nPayload = preparePayloadStopCalibration(transmitPayload);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
						
	mutex1.unlock();
	
	int counter = 0;
	while (true) {
		//
		int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));
		if (n == nPacket) {
			
			if (waitForResponse(payloadIdStopCalibration) == true) {
			
				//std::cout << "stop calibration success" << std::endl;
				return true;
			}
			
		}
		
		if (counter++ > 1000) {
			return false;
		}
		
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return false;
}

std::vector<float> imu::getCalibrationData(uint32_t startAddress, uint32_t endAddress) {
	mutex1.lock();	
	if (connected == false) {
		return std::vector<float>();
	}
	
	outputFlags[payloadIdGetCalibrationData] = false;
	
	int nPayload = preparePayloadGetCalibrationData(transmitPayload, startAddress, endAddress);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
		
	mutex1.unlock();

	int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

	if (n == nPacket) {
		if (waitForResponse(payloadIdGetCalibrationData) == true) {
			
			// 3 * float
			uint32_t size = endAddress - startAddress - 32;
			
			int count = calibrationDataSize_ / (3*4);
                	count = count * (3*4);
			
			std::vector<float> data;
			
			for (int i = 0; i < count; i++) {		
				int k = i % 8; // 8 * 4 (float) = 32 bytes
				// there are 2*3 measurements in one page (32 bytes)
				if (k < 2*3) {
					data.push_back(calibrationData[i]);
				}
			}
			
			return data;
		}
		else {
			return std::vector<float>();
		}
	}
	else {
		return std::vector<float>();
	}
}



uint32_t imu::getCalibrationDataSize() {
	mutex1.lock();
	
	if (connected == false) {
		return false;
	}
	
	outputFlags[payloadIdGetCalibrationDataSize] = false;
	
	
	int nPayload = preparePayloadGetCalibrationDataSize(transmitPayload);
	int nPacket = prepareProtocolPacket(transmitBuffer,
						(unsigned char)('C'),
						transmitPayload, nPayload);
	
	mutex1.unlock();
	int counter = 0;
	while (true) {
		//			
		int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

		if (n == nPacket) {
			if (waitForResponse(payloadIdGetCalibrationDataSize, 1000) == true) {
				return calibrationDataSize;
			}
		}
		
		
		if (counter++ > 1000) {
			return 0;
		}
		
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	}
	
	return 0;
}


// mag. cal.
bool imu::setMagneticFieldCalibrationData_(float* bias_, float* scale_) {

    // scale_ : 3x3 matrix
    
    // set outputs on or off and send to device
    mutex1.lock();

    if (connected == false) {
        return false;
    }

    int nPayload = preparePayloadMagneticFieldCalibrationData(transmitPayload, bias_, scale_);
    int nPacket = prepareProtocolPacket(transmitBuffer,
                                        (unsigned char)('C'),
                                        transmitPayload, nPayload);
    mutex1.unlock();

    int n = serialPtr->write_some(boost::asio::buffer(transmitBuffer, nPacket));

    if (n == nPacket) {
        if (waitForResponse(payloadIdMagneticFieldCalibrationData) == true) {
            if (this->bias_[0] == bias_[0] && this->bias_[1] == bias_[1] && this->bias_[2] == bias_[2] &&
                this->scale_[0] == scale_[0] && this->scale_[1] == scale_[1] && this->scale_[2] == scale_[2] &&
                this->scale_[3] == scale_[3] && this->scale_[4] == scale_[4] && this->scale_[5] == scale_[5] &&
                this->scale_[6] == scale_[6] && this->scale_[7] == scale_[7] && this->scale_[8] == scale_[8]) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool imu::setMagneticFieldCalibrationData(float* bias_, float* scale_) {
    if (connected == false) {
        return false;
    }

    while (true) {
        if (setMagneticFieldCalibrationData_(bias_, scale_) == true) {
            return true;
        }
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }

    return false;
}

/////////////////////////////////////////

void imu::waitForNBytes(const int n) {
	int fd = serialPtr->lowest_layer().native_handle();
	
	while (serialPtr->is_open() == true) {
		int bytesAvailable = 0;
		ioctl(fd, FIONREAD, &bytesAvailable);
		
		if (bytesAvailable >= n) {
			break;
		}
	}
	
	return;
}

int imu::readNBytes(const int n, unsigned char* buffer) {
	if (connected == false) {
		return 0;
	}

	int nRead = serialPtr->read_some(boost::asio::buffer(buffer, n));
	
	return nRead;
}

std::vector<float> imu::getQuaternion() {
	float q_[4];
	if (getQuaternion_(q_) == true) {
		std::vector<float> q { q_[0], q_[1], q_[2], q_[3] }; 
		return q;
	}
	return std::vector<float>();
	
}
std::vector<float> imu::getEuler() {
	float euler_[3];
	if (getEuler_(euler_) == true) {
		std::vector<float> euler { euler_[0], euler_[1], euler_[2] }; 
		return euler;
	}
	return std::vector<float>();
}
float imu::getAltitude() {
	float altitude_;
	if (getAltitude_(&altitude_) == true) {
		return altitude_;
	}
	else {
		return HUGE_VAL;
	}
}
std::vector<float> imu::getAccelerations() {
	float accels_[3];
	if (getAccelerations_(accels_) == true) {
		std::vector<float> accels { accels_[0], accels_[1], accels_[2] }; 
		return accels;
	}
	return std::vector<float>();
}
std::vector<float> imu::getRates() {
	float rates_[3];
	if (getRates_(rates_) == true) {
		std::vector<float> rates { rates_[0], rates_[1], rates_[2] }; 
		return rates;
	}
	return std::vector<float>();
}
std::vector<float> imu::getMagneticField() {
	float mag_[3];
	if (getMagneticField_(mag_) == true) {
		std::vector<float> mags { mag_[0], mag_[1], mag_[2] }; 
		return mags;
	}
	return std::vector<float>();
}
std::vector<float> imu::getMagneticFieldRaw() {
	float magRaw_[3];
	if (getMagneticFieldRaw_(magRaw_) == true) {
		std::vector<float> mags { magRaw_[0], magRaw_[1], magRaw_[2] }; 
		return mags;
	}
	return std::vector<float>();
}
std::vector<float> imu::getCovariances() {
	float cov_[3];
	if (getCovariances_(cov_) == true) {
		std::vector<float> cov { cov_[0], cov_[1], cov_[2] }; 
		return cov;
	}
	return std::vector<float>();
}

std::vector<float> imu::getDebugRaw() {
	float accelsRaw_[3];
	float ratesRaw_[3];
	float magRaw_[3];
	
	if (getDebugRaw_(accelsRaw_, ratesRaw_, magRaw_) == true) {
		std::vector<float> debug { accelsRaw_[0], accelsRaw_[1], accelsRaw_[2], 
					   ratesRaw_[0], ratesRaw_[1], ratesRaw_[2],
					   magRaw_[0], magRaw_[1], magRaw_[2] }; 
		return debug;
	}
	return std::vector<float>();
}
