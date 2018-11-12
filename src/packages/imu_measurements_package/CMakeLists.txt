cmake_minimum_required(VERSION 2.8.3)
project(imu_measurements_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## wiringPi library
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Imu_message.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_package
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS} #wiringPi headers directory
)

## This will create executables of the nodes
add_executable(${PROJECT_NAME}_publisher_node src/imu_measurements_publisher.cpp)
## This will link executables to their corresponding libraries
target_link_libraries(${PROJECT_NAME}_publisher_node ${catkin_LIBRARIES} ${WIRINGPI_LIBRARIES})

## catkin creates several files which are used as flags to determine if something has been built or not. 
## The flags for messages generatedd are always following the same naming convention: "<package_name>_generate_messages_cpp"
## Therefore This will generate message header file before building the target (outout -o)
add_dependencies(${PROJECT_NAME}_publisher_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

#############
## Testing ##
#############
