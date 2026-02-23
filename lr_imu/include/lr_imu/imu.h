/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : IMU driver header
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
  
#include <string>
#include <map>
#include <thread>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

class imu {

protected:
	bool connected = false;
	
	boost::shared_ptr<boost::asio::serial_port> serialPtr;
	boost::asio::io_service io;
	
	std::string portname;
	uint32_t portBaudRate;
	
	boost::shared_ptr<boost::thread> threadPtr;
	
	boost::mutex mutex1;
	
	//

	float q[4] = {0, 0, 0, 1};
	float euler[3] = {0};
	float altitude = 0;
	float accels[3] = {0};
	float rates[3] = {0};
	float mag[3] = {0};
	float magRaw[3] = {0};
	float cov[3] = {0};
	
	float accelsRaw[3] = {0};
	float ratesRaw[3] = {0};

	uint16_t dataRate = 0;
	uint32_t baudRate = 0;
	uint16_t outputSetting = 0; 
	

	uint16_t firmwareVersion = 0;
	uint16_t hardwareVersion = 0;
	uint32_t serialNumber = 0;
	
	float* calibrationData = nullptr;
	uint32_t calibrationDataSize = 0;
	uint32_t calibrationDataSize_ = 0;
	
	bool syncEnabled = false;
	
	float zeroThreshold = 0;
	
	
	// magnetic calib. response vars.
	float scale_[3*3] = {0};
	float bias_[3] = {0};
	
	// output flags
	std::map<int, bool> outputFlags = {
		{payloadIdAngularRate, false},
		{payloadIdLinearAcceleration, false},
		{payloadIdQuaternion, false},
		{payloadIdAltitude, false},
		{payloadIdCovariance, false},
		{payloadIdEulerAngles, false},
		{payloadIdMagneticField, false},
		{payloadIdMagneticFieldRaw, false},
		
		{payloadIdBaudRate, false},
		{payloadIdOutputDataRate, false},
		{payloadIdOutputSetting, false},
		{payloadIdRevertToFactory, false},
		{payloadIdSaveSettings, false},
		
		{payloadIdFirmwareInfo, false},
		{payloadIdSerialNumber, false},
		{payloadIdMagneticFieldCalibrationData, false},
		
		
		{payloadIdStartCalibration, false},
		{payloadIdStopCalibration, false},
		{payloadIdGetCalibrationData, false},
		{payloadIdGetCalibrationDataSize, false},
		
		{payloadIdDebugRaw, false},
		
		{payloadIdZeroThreshold, false},
		{payloadIdDebugConfig, false}
	};
	
	//
	
	void loop();
	
	//
	uint8_t* transmitBuffer = nullptr;
	uint8_t* transmitPayload = nullptr;
	const int maxBufferSize = 4096;
	const int maxPayloadSize = 4096;
	const int maxCalibrationDataSize = 1024;
	
	
	bool loadSettings_();
	bool saveSettings_();
	
	void resetSettingsFlags();
	bool getBaudRate_(uint32_t* baudRate_);
	bool setBaudRate_(const uint32_t baudRate);
	
	bool getDataRate_(uint16_t* dataRate_);
	bool setDataRate_(const int dataRate);
	
	bool getOutputs_(uint16_t* outputBitMask_);
	bool setOutputs_(uint16_t outputBitMask);
	
	bool getQuaternion_(float* q_);
	bool getEuler_(float* euler_);
	bool getAltitude_(float* altitude_);
	bool getAccelerations_(float* accels_);
	bool getRates_(float* rates_);
	bool getMagneticField_(float* mag_);
	bool getMagneticFieldRaw_(float* magRaw_);
	bool getCovariances_(float* cov_);
	
	bool getDebugRaw_(float* accelsRaw_, float* ratesRaw_, float* magRaw_);
	
	bool setMagneticFieldCalibrationData_(float* bias_, float* scale_);
	
	bool setZeroThreshold_(float zeroThreshold);
	bool setDebugConfig_(	float sigma_g,
				float sigma_wg,
				float alpha_g,
				float sigma_a,
				float sigma_ba,
				float alpha_a,
				float sigma_m,
				float sigma_h,
				float alpha_h,
				float sigma_hs,
				float sigma_z);
	
public:
	imu();
	~imu();
	
	void start();
	void stop();
	void spin();
	
	bool connect(const std::string port, const int baudRate, const int dataBits, const int parity, const int stopBits);
	bool disconnect();


	std::vector<float> getQuaternion();
	std::vector<float> getEuler();
	float getAltitude();
	std::vector<float> getAccelerations();
	std::vector<float> getRates();
	std::vector<float> getMagneticField();
	std::vector<float> getMagneticFieldRaw();
	std::vector<float> getCovariances();

	std::vector<float> getDebugRaw();
	
	uint16_t getDataRate();
	bool setDataRate(const int dataRate);
	
	uint32_t getBaudRate();
	bool setBaudRate(const uint32_t baudRate);
	
	bool isSyncEnabled();
	bool setSyncEnabled(const bool syncEnabled);
	
	uint16_t getOutputs();
	bool setOutputs(uint16_t outputBitMask);
	
	
	bool saveSettings();
	
	bool loadSettings();
	

	bool revertToFactorySettings();
	
	
	bool setMagneticFieldCalibrationData(float* bias_, float* scale_);
	
	
	bool startCalibration();
	bool stopCalibration();

	std::vector<float> getCalibrationData(uint32_t startAddress, uint32_t endAddress);
	uint32_t getCalibrationDataSize();
	
	
	bool setZeroThreshold(const float zeroThreshold);
	bool setDebugConfig(	const float sigma_g,
				const float sigma_wg,
				const float alpha_g,
				const float sigma_a,
				const float sigma_ba,
				const float alpha_a,
				const float sigma_m,
				const float sigma_h,
				const float alpha_h,
				const float sigma_hs,
				const float sigma_z);
	//
	
	void waitForNBytes(const int n);
	int readNBytes(const int n, unsigned char* buffer);
	
	bool processPacket(const unsigned char* receiveBuffer, const int n);
	
	//
	// payload ids
	const uint32_t payloadIdAngularRate = 0;
	const uint32_t payloadIdLinearAcceleration = 1;
	const uint32_t payloadIdQuaternion = 2;
	const uint32_t payloadIdAltitude = 3;
	const uint32_t payloadIdCovariance = 4;
	const uint32_t payloadIdEulerAngles = 5;
	const uint32_t payloadIdMagneticField = 6;
	const uint32_t payloadIdBaudRate = 7;
	const uint32_t payloadIdOutputDataRate = 8;
	const uint32_t payloadIdOutputSetting = 9;
	const uint32_t payloadIdRevertToFactory = 10;
	const uint32_t payloadIdSaveSettings = 11;
	const uint32_t payloadIdGetSetting = 12;
	const uint32_t payloadIdSetSetting = 13;
	const uint32_t payloadIdMagneticFieldRaw = 14;
	
	const uint32_t payloadIdMagneticFieldCalibrationData = 15;
	const uint32_t payloadIdFirmwareInfo = 16;
	const uint32_t payloadIdSerialNumber = 17;
	
	const uint32_t payloadIdStartCalibration = 18;
	const uint32_t payloadIdStopCalibration = 19;
	const uint32_t payloadIdGetCalibrationData = 20;
	const uint32_t payloadIdGetCalibrationDataSize = 21;
	
	const uint32_t payloadIdDebugRaw = 22;
	
	const uint32_t payloadIdZeroThreshold = 23;
	const uint32_t payloadIdDebugConfig = 24;
	//
	
	const uint32_t outputSettingAngularRate = 0;
	const uint32_t outputSettingLinearAcceleration = 1;
	const uint32_t outputSettingQuaternion = 2;
	const uint32_t outputSettingAltitude = 3;
	const uint32_t outputSettingCovariance = 4;
	const uint32_t outputSettingEulerAngles = 5;
	const uint32_t outputSettingMagneticField = 6;
	const uint32_t outputSettingSyncOutput = 7;
	const uint32_t outputSettingMagneticFieldRaw = 8;
	const uint32_t outputSettingDebugRaw = 9;
	
	//////////////////////////
	// helper functions

	void floatToBytes(const float var, unsigned char* bytes) {
		unsigned char* ptr = (unsigned char*)(&var);
		bytes[0] = ptr[0];
		bytes[1] = ptr[1];
		bytes[2] = ptr[2];
		bytes[3] = ptr[3];

		return;
	}

	void bytesToUInt16(const uint8_t* b, uint16_t* var) {
		uint8_t* ptr = (uint8_t*)(var); 
		ptr[0] = b[0]; ptr[1] = b[1];
		return;
	}
	
	void bytesToUInt32(const uint8_t* b, uint32_t* var) {
		uint8_t* ptr = (uint8_t*)(var); 
		ptr[0] = b[0]; ptr[1] = b[1];
		ptr[2] = b[2]; ptr[3] = b[3];
		return;
	}
	
	
	void int16toBytes(const int16_t var, unsigned char* bytes) {
		bytes[0] = var & 0xff;
		bytes[1] = (var >> 8) & 0xff;

		return;
	}

	void uint16toBytes(const uint16_t var, unsigned char* bytes) {
		bytes[0] = var & 0xff;
		bytes[1] = (var >> 8) & 0xff;

		return;
	}

	void uint32toBytes(const uint32_t var, unsigned char* bytes) {
		bytes[0] = var & 0xff;
		bytes[1] = (var >> 8) & 0xff;
		bytes[2] = (var >> 16) & 0xff;
		bytes[3] = (var >> 24) & 0xff;

		return;
	}

	void bytesToFloat(const uint8_t* bytes, float* var) {
		uint8_t* ptr = (uint8_t*)(var);

		ptr[0] = bytes[0];
		ptr[1] = bytes[1];
		ptr[2] = bytes[2];
		ptr[3] = bytes[3];

		return;
	}
		
	//////////////////////////
	// protocol functions
	
	int preparePayloadBaudRate(uint8_t* payload, uint32_t baudRate) {

		payload[0] = payloadIdSetSetting; // baud rate
		payload[1] = payloadIdBaudRate; // baud rate
		uint32toBytes(baudRate, &payload[2]);

		return 2 + 4;
	}
	
	int preparePayloadDataRate(uint8_t* payload, uint32_t dataRate) {

		payload[0] = payloadIdSetSetting; // 
		payload[1] = payloadIdOutputDataRate; // data rate
		uint16toBytes(dataRate, &payload[2]);

		return 2 + 2;
	}
	
	int preparePayloadOutputSetting(uint8_t* payload, uint16_t outputSetting) {

		payload[0] = payloadIdSetSetting; // 
		payload[1] = payloadIdOutputSetting; // data rate
		uint16toBytes(outputSetting, &payload[2]);

		return 2 + 2;
	}
	
	int preparePayloadZeroThreshold(uint8_t* payload, float zeroThreshold) {

		payload[0] = payloadIdSetSetting; // 
		payload[1] = payloadIdZeroThreshold; // zero thr.
		
		floatToBytes(zeroThreshold, &payload[2]);

		return 2 + 4;
	}
	
	int preparePayloadDebugConfig(uint8_t* payload, 	
						float sigma_g,
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

		int n = 0;
		payload[n++] = payloadIdSetSetting; // 
		payload[n++] = payloadIdDebugConfig; // zero thr.
		
		floatToBytes(sigma_g, &payload[n]);
		n += 4;
		floatToBytes(sigma_wg, &payload[n]);
		n += 4;
		floatToBytes(alpha_g, &payload[n]);
		n += 4;
		floatToBytes(sigma_a, &payload[n]);
		n += 4;
		floatToBytes(sigma_ba, &payload[n]);
		n += 4;
		floatToBytes(alpha_a, &payload[n]);
		n += 4;
		floatToBytes(sigma_m, &payload[n]);
		n += 4;
		floatToBytes(sigma_h, &payload[n]);
		n += 4;
		floatToBytes(alpha_h, &payload[n]);
		n += 4;
		floatToBytes(sigma_hs, &payload[n]);
		n += 4;
		floatToBytes(sigma_z, &payload[n]);
		n += 4;

		return n;
	}
	
	int preparePayloadSaveSettings(uint8_t* payload) {

		payload[0] = payloadIdSaveSettings; // 
		uint16toBytes(outputSetting, &payload[1]);

		return 1 + 2;
	}
	
	int preparePayloadRevertToFactory(uint8_t* payload) {

		payload[0] = payloadIdRevertToFactory; // 
		uint16toBytes(outputSetting, &payload[1]);

		return 1 + 2;
	}

	int preparePayloadLoadSettings(uint8_t* payload, const std::vector<uint32_t>& payloadIds) {
		int n = 0;
	
		payload[n++] = payloadIdGetSetting; // 
		
		for (auto payloadId : payloadIds) {
			payload[n++] = (uint8_t)(payloadId);
		}

		return n;
	}
	
	int preparePayloadStartCalibration(uint8_t* payload) {
		int n = 0;
	
		payload[n++] = payloadIdStartCalibration; // 
		

		return n;
	}
	int preparePayloadStopCalibration(uint8_t* payload) {
		int n = 0;
	
		payload[n++] = payloadIdStopCalibration; // 
		

		return n;
	}
	
	int preparePayloadGetCalibrationData(uint8_t* payload, uint32_t startAddress, uint32_t endAddress) {
		int n = 0;
	
		payload[n++] = payloadIdGetCalibrationData; // 
		
		uint32toBytes(startAddress, &payload[n]);
		n += 4;
		uint32toBytes(endAddress, &payload[n]);
		n += 4;

		return n;
	}
	int preparePayloadGetCalibrationDataSize(uint8_t* payload) {
		int n = 0;
	
		payload[n++] = payloadIdGetCalibrationDataSize; // 
		

		return n;
	}
	
	int prepareProtocolPacket(uint8_t* sendBuffer,
					uint8_t type,
					uint8_t* payload, uint8_t payloadLength) {
		int n = 0;

		sendBuffer[n++] = 'S'; // sync
		sendBuffer[n++] = type; // packet type
		// packet size
		sendBuffer[n++] = 0; // lo (2)
		sendBuffer[n++] = 0; // hi (3)

		for (int i = 0; i < payloadLength; i++) {
			sendBuffer[n++] = payload[i];
		}

		// checksum
		uint32_t checksum = 0;
		for (int j = 4; j < n; j++) {
			checksum += (uint32_t)(sendBuffer[j]);
		}
		

		uint16_t checksum16 = checksum & 0xffff;
		uint16toBytes(checksum16, &sendBuffer[n]);
		n += 2;

		// packet size
		uint16toBytes((uint16_t)(n), &sendBuffer[2]);

		return n;
	}
	
	
	int preparePayloadMagneticFieldCalibrationData(uint8_t* payload, float* bias_, float* scale_) {

		payload[0] = payloadIdSetSetting; // 
		payload[1] = payloadIdMagneticFieldCalibrationData; 
		
		memcpy(&payload[2], &bias_[0], 3 * sizeof(float));
		memcpy(&payload[2 + 3 * sizeof(float)], &scale_[0], 3*3 * sizeof(float));

		return 1 + 1 + 3 * sizeof(float) + 3*3 * sizeof(float);
	}
	
	
	//
	
	bool waitForResponse(const uint32_t payloadId, int timeout = 50) {
		//const int timeout = 50;
		int timeoutCounter = 0;
		
		bool ready = false;
		
		//std::cout << "outputFlags payloadId = " << payloadId << " waiting" << std::endl;
		while (true) {
			if (++timeoutCounter >= timeout) {
				ready = false;
				break;
			}
			
			
			//std::cout << " " << payloadId << " : " << outputFlags[payloadId] << std::endl;
			if (outputFlags[payloadId] == true) {
				
				ready = true;
				break;
			}
		
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}
		
		if (ready == true) {
			return true;
		}
		else {
			return false;
		}
	}
	
	bool waitForResponses(const std::vector<uint32_t>& payloadIds) {
		const int timeout = 10;
		int timeoutCounter = 0;
		
		bool ready = true;
				
		while (true) {
			ready = true;
			
			/*for (auto payloadId : payloadIds) {
				printf("payload %d = %d \n", payloadId, outputFlags[payloadId]);
			}*/
			
			for (auto payloadId : payloadIds) {
				//printf("payload %d = %d", payloadId, outputFlags[payloadId]);
			
				if (outputFlags[payloadId] == false) {
					ready = false;
					
					break;
				}
			}
			
			if (ready == true) {
				break;
			}
			
			if (++timeoutCounter >= timeout) {
				break;
			}
			
			boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
		}
		
		if (ready == true) {
			return true;
		}
		else {
			return false;
		}
	}
	
	void lock() {
		mutex1.lock();
	}
	void unlock() {
		mutex1.unlock();
	}
	
};


