cmake_minimum_required(VERSION 2.8.3)
project(map_module)

option(ENABLE_DEBUG "Enter debug mode" OFF)
if(ENABLE_DEBUG)
  add_compile_options(-DENABLE_DEBUG)
endif(ENABLE_DEBUG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  message_generation
)

# find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  curvepoint.msg
  curve.msg
  locallane.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  path_planning.srv
  get_curve.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  # nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  message_generation
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(static_curve_node
  src/static_curve_node.cpp
)
add_dependencies(static_curve_node map_module_generate_messages_cpp ${catkin_EXPORTED_TARGETS} )

target_link_libraries(static_curve_node
  ${catkin_LIBRARIES}
)

add_executable(map_server_node
  src/map_server_node.cpp
)
add_dependencies(map_server_node map_module_generate_messages_cpp ${catkin_EXPORTED_TARGETS} )

target_link_libraries(map_server_node
  ${catkin_LIBRARIES}
)

add_executable(get_curve_client_node
  src/get_curve_client.cpp
)

add_dependencies(get_curve_client_node map_module_generate_messages_cpp ${catkin_EXPORTED_TARGETS} )

target_link_libraries(get_curve_client_node
  ${catkin_LIBRARIES}
)

add_executable(basic_shapes
  src/line_vis.cpp
)
target_link_libraries(basic_shapes
  ${catkin_LIBRARIES}
)
# install(TARGETS static_curve_node
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# install(DIRECTORY params/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)

# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
