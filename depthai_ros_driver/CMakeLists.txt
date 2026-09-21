cmake_minimum_required(VERSION 3.22)
project(depthai_ros_driver_v3 VERSION 3.3.0)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_SHARED_LIBS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations
                    -Wno-maybe-uninitialized)
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(backward_ros REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

set(COMMON_LIB_NAME ${PROJECT_NAME}_common)
add_library(
  ${COMMON_LIB_NAME} SHARED
  src/utils.cpp
  src/dai_nodes/base_node.cpp
  src/dai_nodes/sys_logger.cpp
  src/dai_nodes/sensors/img_pub.cpp
  src/dai_nodes/sensors/sensor_helpers.cpp
  src/param_handlers/driver_param_handler.cpp
  src/param_handlers/imu_param_handler.cpp
  src/param_handlers/nn_param_handler.cpp
  src/param_handlers/sensor_param_handler.cpp
  src/param_handlers/feature_tracker_param_handler.cpp
  src/param_handlers/rgbd_param_handler.cpp
  src/param_handlers/vio_param_handler.cpp
  # src/param_handlers/slam_param_handler.cpp
  src/param_handlers/stereo_param_handler.cpp
  src/param_handlers/thermal_param_handler.cpp
  src/param_handlers/tof_param_handler.cpp
  src/param_handlers/pipeline_gen_param_handler.cpp
  src/param_handlers/sync_param_handler.cpp)

target_link_libraries(
  ${COMMON_LIB_NAME}
  PUBLIC ${COMMON_DEPS}
  camera_info_manager::camera_info_manager
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  depthai_bridge_v3::depthai_bridge_v3
  depthai_v3::core
  image_transport::image_transport
  rclcpp::rclcpp
  diagnostic_updater::diagnostic_updater
  ${diagnostic_msgs_TARGETS}
  ${ffmpeg_image_transport_msgs_TARGETS})

set(SENSOR_LIB_NAME ${PROJECT_NAME}_sensor_nodes)
add_library(
  ${SENSOR_LIB_NAME} SHARED
  src/dai_nodes/sensors/imu.cpp
  src/dai_nodes/sensors/camera.cpp
  src/dai_nodes/sensors/feature_tracker.cpp
  src/dai_nodes/sensors/sensor_wrapper.cpp
  src/dai_nodes/sensors/stereo.cpp
  src/dai_nodes/sensors/thermal.cpp
  src/dai_nodes/sensors/rgbd.cpp
  src/dai_nodes/sensors/vio.cpp
  # src/dai_nodes/sensors/slam.cpp
  src/dai_nodes/sensors/tof.cpp
  src/dai_nodes/sensors/sync.cpp)

target_link_libraries(
  ${SENSOR_LIB_NAME}
  PUBLIC
  ${SENSOR_DEPS}
  ${COMMON_LIB_NAME}
  cv_bridge::cv_bridge
  ${sensor_msgs_TARGETS}
  image_transport::image_transport
  camera_info_manager::camera_info_manager
  depthai_v3::core
  depthai_bridge_v3::depthai_bridge_v3
  rclcpp::rclcpp
  diagnostic_updater::diagnostic_updater
  ${geometry_msgs_TARGETS}
  ${depthai_ros_msgs_v3_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros)

set(NN_LIB_NAME ${PROJECT_NAME}_nn_nodes)
add_library(
  ${NN_LIB_NAME} SHARED
  src/dai_nodes/nn/nn_wrapper.cpp src/dai_nodes/nn/spatial_nn_wrapper.cpp
  src/dai_nodes/nn/segmentation.cpp)

target_link_libraries(
  ${NN_LIB_NAME}
  PUBLIC ${COMMON_LIB_NAME} depthai_v3::core depthai_bridge_v3::depthai_bridge_v3
  rclcpp::rclcpp ${vision_msgs_TARGETS})

add_library(
  ${PROJECT_NAME} SHARED
  src/driver.cpp src/pipeline/pipeline_generator.cpp
  src/pipeline/base_pipeline.cpp src/pipeline/base_types.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
  rclcpp::rclcpp
  depthai_v3::core
  ${rclcpp_components_TARGETS}
  ${std_srvs_TARGETS}
  ${pluginlib_TARGETS})
add_executable(driver_node src/driver_node.cpp)
target_link_libraries(driver_node PUBLIC ${PROJECT_NAME} ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME} ${COMMON_LIB_NAME})

# ---------------------------------------------------------------------------
# NoMagic: optional native ROS1 ("ROS One") support. One guarded block + the
# cmake/ and src/ros1/ files + a few #ifdef NOMAGIC_ROS1 hooks; upstream builds
# with the option OFF are unchanged. See docs/NOMAGIC_ROS1.md.
option(NOMAGIC_ROS1 "Also publish images and serve trigger services on ROS1" OFF)
if(NOMAGIC_ROS1)
  # find_ros1_package() needs the ROS1 setup.bash sourced (PKG_CONFIG_PATH).
  include(cmake/find_ros1_package.cmake)
  find_ros1_package(roscpp REQUIRED)
  find_ros1_package(std_srvs REQUIRED)
  find_ros1_package(sensor_msgs REQUIRED)
  # All ROS1 code is confined to one helper library: the flat ROS1 include tree
  # (/opt/ros/one/include) shadows same-named ROS2 headers (pluginlib, tf2_ros,
  # class_loader, ...), so its include dirs must stay PRIVATE to the single
  # translation unit ros1_node.cpp (the public ros1_node.hpp is ROS1-free).
  set(ROS1_LIB_NAME ${PROJECT_NAME}_nomagic_ros1)
  add_library(${ROS1_LIB_NAME} SHARED src/ros1/ros1_node.cpp)
  target_include_directories(${ROS1_LIB_NAME} SYSTEM PRIVATE
    ${ros1_roscpp_INCLUDE_DIRS}
    ${ros1_std_srvs_INCLUDE_DIRS}
    ${ros1_sensor_msgs_INCLUDE_DIRS})
  target_link_libraries(${ROS1_LIB_NAME}
    PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS}
    PRIVATE ${ros1_roscpp_LIBRARIES} ${ros1_std_srvs_LIBRARIES} ${ros1_sensor_msgs_LIBRARIES})
  # PUBLIC define: NOMAGIC_ROS1 changes the ImagePublisher class layout in
  # img_pub.hpp, so every dependent target must see it.
  target_compile_definitions(${COMMON_LIB_NAME} PUBLIC NOMAGIC_ROS1=1)
  target_link_libraries(${COMMON_LIB_NAME} PUBLIC ${ROS1_LIB_NAME})
  # Force RPATH instead of RUNPATH so the link-time library directories (both
  # /opt/ros/humble/lib and the ROS1 lib dir) win over LD_LIBRARY_PATH no matter
  # which stack was sourced last (same-soname libraries exist in both distros).
  foreach(_t ${ROS1_LIB_NAME} ${COMMON_LIB_NAME} ${SENSOR_LIB_NAME} ${NN_LIB_NAME} ${PROJECT_NAME} driver_node)
    target_link_options(${_t} PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(${_t} PROPERTIES
      INSTALL_RPATH_USE_LINK_PATH TRUE
      BUILD_RPATH_USE_ORIGIN TRUE
      SKIP_BUILD_RPATH FALSE)
  endforeach()
  install(
    TARGETS ${ROS1_LIB_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endif()
# ---------------------------------------------------------------------------

rclcpp_components_register_nodes(${PROJECT_NAME} PLUGIN
  "depthai_ros_driver::Driver" EXECUTABLE driver)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(camera_info_manager cv_bridge depthai_bridge_v3 depthai_v3
  diagnostic_updater std_srvs)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${SENSOR_LIB_NAME} ${NN_LIB_NAME} ${COMMON_LIB_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(EXPORT ${PROJECT_NAME}Targets DESTINATION share/${PROJECT_NAME}/cmake)
install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)

install(DIRECTORY include/ DESTINATION include)
if(BUILD_TESTING AND TEST_DEPTHAI_ROS_DRIVER)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
      set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
      add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_driver.py)
  add_ros_isolated_launch_test(test/test_rgbd_pcl.py)
  add_ros_isolated_launch_test(test/test_detections.py)
  add_ros_isolated_launch_test(test/test_spatial_detections.py)
  add_ros_isolated_launch_test(test/test_rs_compat.py)
  add_ros_isolated_launch_test(test/test_compressed_mjpg.py)
  add_ros_isolated_launch_test(test/test_low_bandwidth.py)
  add_ros_isolated_launch_test(test/test_compressed_ffmpeg.py)
  add_ros_isolated_launch_test(test/test_neural_depth.py)
  add_ros_isolated_launch_test(test/test_segmentation.py)
  add_ros_isolated_launch_test(test/test_vio.py)
endif()
ament_package()
