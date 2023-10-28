cmake_minimum_required(VERSION 3.0)
project(fingervision)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  fingervision_msgs
  std_msgs
  std_srvs
  image_transport
  camera_info_manager
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

set(PROJECT_BASE_DIR ${PROJECT_SOURCE_DIR}/..)
set(FV_CORE_DIR ${PROJECT_BASE_DIR}/fv_core)
set(THIRDPARTY_DIR ${PROJECT_BASE_DIR}/3rdparty)

# ++++OPENCV-3.x++++
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ++++OPENCV-2.x++++
# find_package(OpenCV 2 REQUIRED)
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ++++OPENCV-2.4.13++++
# # NOTE: Install OpenCV 2.4.13 on ~/.local from source which includes cv::fisheye
# # We don't use opencv found by any prior running of find_package
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
# set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
# find_package(OpenCV 2.4.13 REQUIRED)
# # restore CMAKE_PREFIX_PATH and other cached variables
# # in order to avoid other packages find this opencv.
# set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ----OPENCV----

catkin_package(
  LIBRARIES
    fv_blob_tracker2
    fv_prox_vision
    ay_vision
  CATKIN_DEPENDS
    roscpp
    fingervision_msgs
    std_msgs
    std_srvs
    image_transport
    camera_info_manager
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${FV_CORE_DIR}
  ${THIRDPARTY_DIR}
)

add_library(fv_blob_tracker2 ${FV_CORE_DIR}/blob_tracker2.cpp)
target_link_libraries(fv_blob_tracker2 ${OpenCV_LIBRARIES})

add_library(fv_prox_vision ${FV_CORE_DIR}/prox_vision.cpp)
target_link_libraries(fv_prox_vision ${OpenCV_LIBRARIES})

add_library(ay_vision ${THIRDPARTY_DIR}/ay_vision/vision_util.cpp)
target_link_libraries(ay_vision ${OpenCV_LIBRARIES})

add_executable(fv_core_node src/fv_core_node.cpp)
add_dependencies(fv_core_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fv_core_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  fv_blob_tracker2
  fv_prox_vision
  ay_vision
  # libcv_bridge.so
  boost_thread
  boost_system
)
# NOTE: Adding libcv_bridge.so is a quick hack.
# In manifest.xml cv_bridge is omitted.
# If we put cv_bridge as a dependency in manifest.xml,
# both opencv 2.4.8 and 2.4.13 will be linked simultaneously,
# which causes a compile issue.
