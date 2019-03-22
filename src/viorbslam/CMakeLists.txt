cmake_minimum_required(VERSION 2.8.3)
project(viorbslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()


MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()
## Find catkin macros and libraries
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  compressed_depth_image_transport
  compressed_image_transport
  cv_bridge
  image_transport
  message_filters
  kinect2_bridge
  roscpp
  rostime
  rosbag
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package( G2O REQUIRED )
find_package(OpenMP)
find_package(Pangolin 0.1 REQUIRED)
find_package(PCL 1.7 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

include_directories(/usr/include/openni2)
include_directories(/usr/include/ni)
include_directories(/usr/include/qt4)
include_directories(/usr/local/include/opencv)
include_directories(/usr/include/qt4/Qt)
include_directories(/usr/include/qt4/QtCore)
include_directories(/usr/include/qt4/QtGui)
include_directories(/usr/include/qt4/QtOpenGL)
include_directories(usr/include)
include_directories(/usr/local/include)
include_directories(/usr/include/boost)
include_directories(/usr/include/eigen3)
include_directories(/usr/include/flann)
include_directories(/usr/include/qhull)
include_directories(/usr/local/include/g2o)
include_directories(/home/jinglun/fusion_ws/src/viorbslam/Thirdparty/g2o/g2o)
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${VTK_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(ros_rgbd src/ros_rgbd.cc)

add_library(orbslam SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/pointcloudmapping.cc
)

target_link_libraries(ros_rgbd
	orbslam
    ${BOOST_LIBRARY}
    ${OPENNI_LIBRARY}
    ${VTK_LIBRARIES}
    ${catkin_LIBRARIES}
 	${OpenCV_LIBRARIES}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
  	${PCL_LIBRARIES}
  	${kinect2_bridge_LIBRARIES}
 	${Pangolin_LIBRARIES}
  	${DBoW_LIBRARY}
	${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)


