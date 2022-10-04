cmake_minimum_required(VERSION 3.0.2)
project(stvo_pl_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
  # image_transport
  roscpp
  std_msgs
  # sensor_msgs
  visualization_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
find_package(yaml-cpp REQUIRED CONFIG PATHS ${YAML_PATHS})

find_package(OpenCV 4 REQUIRED)
link_directories(${OpenCV_LIBS_DIR})
include_directories(${OpenCV2_INCLUDE_DIRS})


catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories("/usr/include/eigen3")
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/3rdparty/line_descriptor/include/
)
message(STATUS ${PROJECT_SOURCE_DIR})

list(APPEND SRC
  src/auxiliar.cpp
  src/lineIterator.cpp
  src/config.cpp
  src/dataset.cpp
  src/gridStructure.cpp
  src/matching.cpp
  src/pinholeStereoCamera.cpp
  src/stereoFeatures.cpp
  src/stereoFrame.cpp
  src/stereoFrameHandler.cpp
  src/timer.cpp
  src/visualizer.cpp
)

add_executable(${PROJECT_NAME}_node app/imageStVO.cpp ${SRC})

target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
./3rdparty/line_descriptor/lib/liblinedesc.so
)
