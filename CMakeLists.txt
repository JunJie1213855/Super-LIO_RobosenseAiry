cmake_minimum_required(VERSION 3.16)
project(super_lio_slam VERSION 2.0 LANGUAGES CXX C)

# LIO must keep up with high-rate IMU and large point clouds during bag replay.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudPose.msg"
  "msg/CloudPose2.msg"
  "msg/KeyFrame.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(basic_core
  basic/src/Manifold.cpp
  basic/src/logs.cpp
)
target_include_directories(basic_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/basic/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/basic>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(basic_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} glog gflags)

add_library(std_descriptor_core std/src/STDesc.cpp)
target_include_directories(std_descriptor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/std/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/std>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(std_descriptor_core PUBLIC
  rclcpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions)
target_link_libraries(std_descriptor_core PUBLIC
  Eigen3::Eigen ${PCL_LIBRARIES} Ceres::ceres OpenMP::OpenMP_CXX)

add_library(lio
  src/lio/ESKF.cpp
  src/lio/params.cpp
  src/lio/super_lio.cpp
  src/lio/super_lio_reloc.cpp
  src/ros/ROSWrapper.cpp
)
target_include_directories(lio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(lio PUBLIC ROOT="${CMAKE_CURRENT_SOURCE_DIR}/")
ament_target_dependencies(lio PUBLIC
  rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs nav_msgs
  pcl_ros pcl_conversions livox_ros_driver2 tf2_ros rosidl_default_runtime)
target_link_libraries(lio PUBLIC
  basic_core Eigen3::Eigen ${PCL_LIBRARIES} glog
  ${cpp_typesupport_target})

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node PRIVATE lio)

add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node PRIVATE lio)

add_executable(std_extract_node std/ros2/std_extract_node.cpp)
ament_target_dependencies(std_extract_node
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_conversions)
target_link_libraries(std_extract_node std_descriptor_core)

add_executable(loop_backend_node
  src/apps/loop_backend_node.cpp
  src/loop/LoopBackendNode.cpp
)
target_include_directories(loop_backend_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(loop_backend_node
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs
  pcl_conversions tf2_ros rosidl_default_runtime)
target_link_libraries(loop_backend_node
  std_descriptor_core Eigen3::Eigen ${PCL_LIBRARIES} gtsam
  ${cpp_typesupport_target})

install(TARGETS basic_core std_descriptor_core lio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  super_lio_node relocation_node std_extract_node loop_backend_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY basic/include/ DESTINATION include/${PROJECT_NAME}/basic)
install(DIRECTORY std/include/ DESTINATION include/${PROJECT_NAME}/std)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
