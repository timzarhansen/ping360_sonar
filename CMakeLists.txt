cmake_minimum_required(VERSION 3.3)
project(ping360_sonar)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CATKIN_ENABLE_TESTING OFF)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

ament_python_install_package(ping360_sonar)

add_executable(visualizationPing360 src/visualizationSonarImage.cpp)
ament_target_dependencies(visualizationPing360 rclcpp std_msgs sensor_msgs ping360_sonar_msgs cv_bridge)

install(PROGRAMS
        nodes/ping360_node
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
        visualizationPing360
        DESTINATION lib/${PROJECT_NAME}
)



ament_package()

