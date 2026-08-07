cmake_minimum_required(VERSION 3.16)
project(hik_camera_driver VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(HikMVS REQUIRED)

add_library(hik_camera_device SHARED
  src/camera_device.cpp
)
target_include_directories(hik_camera_device
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(hik_camera_device PUBLIC HikMVS::MvCameraControl)
set_target_properties(hik_camera_device PROPERTIES BUILD_RPATH "${HIK_MVS_LIBRARY_DIR}")
if(HIK_MVS_INSTALL_RUNTIME)
  set_target_properties(hik_camera_device PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()
target_compile_options(hik_camera_device PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(hik_camera_component SHARED
  src/hik_camera_node.cpp
)
target_include_directories(hik_camera_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(hik_camera_component hik_camera_device)
ament_target_dependencies(hik_camera_component
  camera_info_manager
  diagnostic_msgs
  image_transport
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs
  std_msgs
  std_srvs
)
target_compile_options(hik_camera_component PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

rclcpp_components_register_node(hik_camera_component
  PLUGIN "hik_camera_driver::HikCameraNode"
  EXECUTABLE hik_camera_driver_node
)

install(
  TARGETS hik_camera_device hik_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES cmake/FindHikMVS.cmake DESTINATION share/${PROJECT_NAME}/cmake)

if(HIK_MVS_INSTALL_RUNTIME)
  install(
    DIRECTORY "${HIK_MVS_LIBRARY_DIR}/"
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so*"
  )
endif()

ament_package()
