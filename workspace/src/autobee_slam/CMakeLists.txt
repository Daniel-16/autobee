cmake_minimum_required(VERSION 3.16)
project(autobee_slam CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ORB-SLAM3 lives outside the colcon workspace -- it is built by
# scripts/build_orbslam3.sh into /opt/autobee/third_party/ORB_SLAM3.
# Override with -DORB_SLAM3_ROOT=... if you put it elsewhere.
set(ORB_SLAM3_ROOT "/opt/autobee/third_party/ORB_SLAM3" CACHE PATH
  "Root of the locally-built ORB-SLAM3 source tree")

if(NOT EXISTS "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR
    "ORB-SLAM3 library not found at ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so. "
    "Run scripts/build_orbslam3.sh first, or pass -DORB_SLAM3_ROOT=<path>.")
endif()

# Pangolin is a private dep of ORB-SLAM3; we just need to find its headers
# for the include cascade. Installed by build_orbslam3.sh into /usr/local.
find_package(Pangolin REQUIRED)

include_directories(
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable(orb_slam3_node src/orb_slam3_node.cpp)

target_link_libraries(orb_slam3_node
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(orb_slam3_node
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport
)

# rpath so the executable finds libORB_SLAM3.so + thirdparty .so files at runtime
set_target_properties(orb_slam3_node PROPERTIES
  INSTALL_RPATH
    "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS orb_slam3_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
