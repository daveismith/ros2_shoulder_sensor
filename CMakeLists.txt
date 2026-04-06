cmake_minimum_required(VERSION 3.8)
project(ros2_shoulder_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# build library
add_library(${PROJECT_NAME}
  src/magnetic_angle_sensor.cpp
  src/magnetic_angle_sensor_core.cpp
)
  target_sources(${PROJECT_NAME} PRIVATE
    third_party/libcanard/canard.c
    third_party/o1heap/o1heap.c
  )
add_library(${PROJECT_NAME}::magnetic_angle_sensor ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include:${PROJECT_NAME}>)
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

# Suppress pedantic warnings in vendored C libraries (they are not our code)
set_source_files_properties(
  third_party/libcanard/canard.c
  third_party/o1heap/o1heap.c
  PROPERTIES COMPILE_FLAGS "-w"
)

# export plugin description for pluginlib
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    third_party/libcanard/canard.c
    third_party/libcanard/canard.h
    third_party/o1heap/o1heap.c
    third_party/o1heap/o1heap.h
    third_party/cavl2.h
  )
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Vendored third-party C sources do not follow this repository's uncrustify profile.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_magnetic_angle_sensor_core
    test/test_magnetic_angle_sensor_core.cpp
  )
  if(TARGET test_magnetic_angle_sensor_core)
    target_link_libraries(test_magnetic_angle_sensor_core ${PROJECT_NAME})
    target_include_directories(test_magnetic_angle_sensor_core PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(${PROJECT_NAME})

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)


ament_package()
