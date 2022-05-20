cmake_minimum_required(VERSION 3.0.2)
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
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        rospy
        sensor_msgs
        std_msgs
        dynamic_reconfigure
        message_generation
        roslint
        roscpp
        genmsg
        commonbluerovmsg
)
find_package(OpenCV REQUIRED)
catkin_python_setup()
roslint_python()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SonarEcho.msg
)

add_service_files(
        FILES
        sendingSonarConfig.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs#   std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/sonar.cfg
  #...
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sonar-ping360
#  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS message_runtime
        #  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)



install(PROGRAMS
     nodes/ping360_node
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(conversionSonarData src/createImageOfSonarData.cpp)
target_link_libraries(conversionSonarData ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(conversionSonarData ping360_sonar_generate_messages_cpp)

