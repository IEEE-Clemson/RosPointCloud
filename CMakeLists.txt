cmake_minimum_required(VERSION 3.5)
project(mynode)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3)
find_package(PCL REQUIRED)

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

include_directories(
	include
  #  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  #${OCTOMAP_INCLUDE_DIRS}
  # ${OPENCV_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  #${CERES_LIBRARY_DIRS}
)


add_executable(mynode src/main.cpp src/pointcloudodom.cpp include/mynode/main.h include/mynode/pointcloudodom.h)
ament_target_dependencies(mynode
	Eigen3
	PCL
	geometry_msgs
	nav_msgs
	sensor_msgs
	rclcpp
	rosbag2
	std_msgs
	tf2
	pcl_conversions
	tf2_eigen)
target_link_libraries(mynode ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
    mynode
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
