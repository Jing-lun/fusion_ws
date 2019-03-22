cmake_minimum_required(VERSION 2.8.3)
project(visual_imu_fusion)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  nav_msgs)

find_package(Eigen REQUIRED COMPONENTS)

catkin_package(
  DEPENDS Eigen
  INCLUDE_DIRS include
  LIBRARIES visual_imu_fusion
  CATKIN_DEPENDS message_filters roscpp sensor_msgs std_msgs tf nav_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OPENCV_LIBRARIES}
)

## Declare a cpp library
add_library(visual_imu_fusion
  src/visual_imu_fusion.cpp
  src/visual_imu_fusion_node.cpp
  src/visual_imu_fusion_ros.cpp
  include/visual_imu_fusion/visual_imu_fusion.h
  include/visual_imu_fusion/visual_imu_fusion_ros.h
)


# create complementary_filter_node executable
add_executable(visual_imu_fusion_node
  src/visual_imu_fusion_node.cpp)
target_link_libraries(visual_imu_fusion_node visual_imu_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OPENCV_LIBRARIES})

install(TARGETS visual_imu_fusion visual_imu_fusion_node
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
