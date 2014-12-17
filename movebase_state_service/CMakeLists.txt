cmake_minimum_required(VERSION 2.8.3)
project(movebase_state_service)

add_definitions(-std=c++0x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    sensor_msgs
                    std_msgs
                    tf
                    costmap_2d
                    navfn
                    nav_core
                    cv_bridge
                    nav_msgs
                    geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
#~ add_message_files(
  #~ DEPENDENCIES actionlib_msgs std_msgs
#~ )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
  FILES
  MovebaseStateService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    sensor_msgs
#    actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_layer overwrite_layer smart_obstacle_layer unleash_static_planner static_planner 
#  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp nav_core actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)

## Declare a cpp library
#add_library(noise_voxel_grid src/noise_voxel_grid.cpp)
#add_library(noise_approximate_voxel_grid src/noise_approximate_voxel_grid.cpp)

## Declare a cpp executable
add_executable(movebase_state_service src/movebase_state_service.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(scitos_2d_navigation_node scitos_2d_navigation_generate_messages_cpp)
add_dependencies(movebase_state_service movebase_state_service_generate_messages_cpp)
target_link_libraries(movebase_state_service
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

## Mark executables and/or libraries for installation
install(TARGETS movebase_state_service
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
