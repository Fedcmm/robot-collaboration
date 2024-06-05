cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_executable(plugin_client src/plugin_client.cpp)
target_include_directories(plugin_client PUBLIC
${GAZEBO_INCLUDE_DIRS}
${gazebo_ros_INCLUDE_DIRS}
${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(
  plugin_client
  "gazebo_dev"
  "gazebo_ros"
 )
target_link_libraries(plugin_client ${GAZEBO_LIBRARIES})
