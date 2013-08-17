cmake_minimum_required(VERSION 2.8.3)
project(oculus_driver)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs message_generation tf oculus_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES oculus_driver
   CATKIN_DEPENDS roscpp geometry_msgs tf oculus_msgs
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

add_executable(oculus_node src/oculus_node.cpp)
add_dependencies(oculus_ros oculus_ros_gencpp oculus_msgs_gencpp)
add_dependencies(oculus_node oculus_ros)

IF(APPLE)
  target_link_libraries(oculus_ros
    ${IOSURFACE_LIBRAY} ${CARBON_LIBRARY} ${IOKIT_LIBRARY})
ELSE(APPLE)
  target_link_libraries(oculus_ros udev Xinerama)
ENDIF(APPLE)

target_link_libraries(oculus_ros ovr ${catkin_LIBRARIES})
target_link_libraries(oculus_node oculus_ros)


#############
## Install ##
#############

install(TARGETS oculus_ros oculus_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(util-test test/util-test.cpp)
if(TARGET util-test)
  target_link_libraries(util-test oculus_ros)
endif()
