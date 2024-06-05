cmake_minimum_required(VERSION 3.8)
project(join_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED system filesystem)

add_library(join_plugin SHARED src/join_plugin.cpp)
target_compile_features(join_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(join_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${Boost_INCLUDE_DIRS})
ament_target_dependencies(
  join_plugin
  "gazebo_dev"
  "gazebo_ros"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(join_plugin PRIVATE "JOIN_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS join_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  join_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
