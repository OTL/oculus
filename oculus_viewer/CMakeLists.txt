cmake_minimum_required(VERSION 2.8.3)
project(oculus_viewer)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport oculus_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES oculus_viewer
   CATKIN_DEPENDS roscpp oculus_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} src/distort_image.cpp src/viewer.cpp)
add_executable(image_distort_viewer src/image_distort_viewer.cpp)
add_dependencies(image_distort_viewer ${PROJECT_NAME} oculus_msgs_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(image_distort_viewer ${catkin_LIBRARIES} ${PROJECT_NAME})


#############
## Install ##
#############

install(TARGETS image_distort_viewer ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

