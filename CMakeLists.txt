cmake_minimum_required(VERSION 3.0.2)
project(payload_transport_controller)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "-Werror -Wall -Wextra -Wpedantic")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_filters
                                        mavros_msgs roscpp)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if your
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  payload_transport_controller
  CATKIN_DEPENDS
  geometry_msgs
  mavros_msgs
  roscpp
  # DEPENDS Eigen3
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_library(INTERNAL_LIBS "src/${PROJECT_NAME}/internal/geo_functions.cpp")

target_link_libraries(INTERNAL_LIBS PRIVATE Eigen3::Eigen
                                            ${GeographicLib_LIBRARIES})

# Declare a C++ library
add_library(
  ${PROJECT_NAME}
  "src/${PROJECT_NAME}/thrust_tracking_controller.cpp"
  "src/${PROJECT_NAME}/controller_ros_client.cpp"
  "src/${PROJECT_NAME}/payload_transport_controller.cpp")

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen
                      INTERNAL_LIBS)

add_executable(${PROJECT_NAME}_node src/payload_transport_controller_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# ##############################################################################
# Install ##
# ##############################################################################

# all install targets should use catkin DESTINATION variables See
# http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation in contrast to
# setup.py, you can choose the destination catkin_install_python(PROGRAMS
# scripts/my_python_script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark executables for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark libraries for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Mark cpp header files for installation install(DIRECTORY
# include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES # myfile1 # myfile2 DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION} )

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test
# test/test_payload_transport_controller.cpp) if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}) endif()

# Add folders to be run by python nosetests catkin_add_nosetests(test)
