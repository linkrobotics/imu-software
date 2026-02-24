# Link Robotics IMU Software

This repository includes GUI and ROS2 driver of the IMU.

Please calibrate IMU before first use by using the magnetometer calibration section in the main GUI.

## Requirements

- Ubuntu 22.04

- Boost 1.74

- Qt6

- ROS2 humble


## Description of the folders:

   `imu-gui`:	Qt GUI for configuring the IMU hardware (requires Qt6 installed)
   
   <img width="1850" height="1053" alt="gui" src="https://github.com/user-attachments/assets/27fdf3d3-889a-4795-a121-b8a9c4d1e43a" />
   
   `lr_imu`:	ROS2 driver (for humble)


## Installation steps

1. **Installation of the Qt GUI (Optional)**
 
   Follow the steps below to install and run the GUI.

   1.1. Define Qt library env. variables
   ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/Qt/6.9.2/gcc_64/lib```

   1.2. Change directory to bin 
   ```/path/to/imu/bin```

   1.3. Run the executable
   ```./IMU-GUI```

3. **Compile and install the ROS2 driver**

   2.1. Check whether ROS2 humble is installed

   2.2. Copy the files under `lr_imu` into your ros2 workspace

   2.3. Change directory to ros2 workspace
   ```cd ros2_ws```

   2.4. Build
   ```colcon build```

   2.5. Published topics:
   - ```/lr/imu/data``` : Quaternion, accelerations, angular velocities
   - ```/lr/imu/euler_angles``` : Euler angles (roll, pitch, yaw)

   2.5. Start the driver node
   ```ros2 launch lr_imu lr_imu.xml```


