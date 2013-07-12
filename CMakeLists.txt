cmake_minimum_required(VERSION 2.8.3)
project(oculus_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(FILES HMDInfo.msg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES oculus_ros
   CATKIN_DEPENDS roscpp geometry_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  LibOVR/Include
  )

IF(APPLE)
  FIND_LIBRARY(IOSURFACE_LIBRARY IOSurface)
  FIND_LIBRARY(CARBON_LIBRARY Carbon)
  FIND_LIBRARY(IOKIT_LIBRARY IOKit)
  link_directories(LibOVR/Lib/MacOS/Debug)
ELSE(APPLE)
  IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    link_directories(LibOVR/Lib/Linux/Debug/x86_64)
  ELSE("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    link_directories(LibOVR/Lib/Linux/Debug/i386)
  ENDIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
ENDIF(APPLE)

add_library(oculus_ros
  src/oculus_ros.cpp
  src/util.cpp
  )

add_executable(oculus_ros_node src/oculus_ros_node.cpp)

add_dependencies(oculus_ros oculus_ros_gencpp)
add_dependencies(oculus_ros_node oculus_ros)

IF(APPLE)
  target_link_libraries(oculus_ros
    ${IOSURFACE_LIBRAY} ${CARBON_LIBRARY} ${IOKIT_LIBRARY})
ELSE(APPLE)
  target_link_libraries(oculus_ros udev Xinerama)
ENDIF(APPLE)
target_link_libraries(oculus_ros ovr ${catkin_LIBRARIES})
target_link_libraries(oculus_ros_node oculus_ros)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS oculus_ros oculus_ros_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(util-test test/util-test.cpp)
if(TARGET util-test)
  target_link_libraries(util-test oculus_ros)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
