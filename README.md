# Link Robotics IMU Software

Please calibrate IMU before first use by using magnetometer calibration in GUI.



Description of the folders:

   `imu-gui`:	Qt GUI for configuring the IMU hardware (requires Qt6 installed)

   `lr_imu`:	ROS2 driver (for humble)

## Installation steps

1. **Installation of the Qt GUI (Optional)**
 
   Follow the steps below to install and run the GUI.

   1.1. Define Qt library variables
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

   2.5. Start the driver node
   ```ros2 launch lr_imu lr_imu.xml```

   2.6. Start the ROS calibration node (optional)
   ```ros2 launch lr_imu calib.xml```

