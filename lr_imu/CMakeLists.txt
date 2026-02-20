cmake_minimum_required(VERSION 3.5)
project(lr_imu)

set(CMAKE_BUILD_TYPE Release)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=10000000")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono thread)
find_package(Eigen3 REQUIRED)
# set(EIGEN_INCLUDE_DIRS "/usr/local/include/eigen3")

#set(BOOST_INCLUDE_DIRS "/usr/local/include/boost")
#set(BOOST_LIBRARIES "/usr/local/lib/libboost_chrono.so" "/usr/local/lib/libboost_thread.so")

# driver
add_executable(lr_imu src/driver.cpp src/imu.cpp)
ament_target_dependencies(lr_imu rclcpp sensor_msgs)

include_directories(include/lr_imu ${Boost_INCLUDE_DIRS})
target_link_libraries(lr_imu Boost::chrono Boost::thread)

# calib
add_executable(lr_imu_calib src/calib.cpp src/imu.cpp)
ament_target_dependencies(lr_imu_calib rclcpp sensor_msgs)

include_directories(include/lr_imu ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(lr_imu_calib Boost::chrono Boost::thread)

#

install(TARGETS
  lr_imu
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  lr_imu_calib
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
