cmake_minimum_required(VERSION 2.8.3)
project(apc2)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    message_generation
    moveit_core
    moveit_ros_planning_interface
    image_transport
    cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_message_files(
    FILES
    Status.msg
)

add_service_files(
    FILES
    recognize.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS moveit_core moveit_ros_planning_interface geometry_msgs roscpp rospy image_transport cv_bridge std_msgs message_runtime message_generation
    DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(save src/save.cpp)
add_executable(find src/find.cpp)
add_executable(pick src/pick.cpp)
add_executable(move src/move.cpp)

add_dependencies(find apc_generate_messages_cpp)

target_link_libraries(save
    ${catkin_LIBRARIES}
)
target_link_libraries(find
    ${catkin_LIBRARIES}
)
target_link_libraries(pick
    ${catkin_LIBRARIES}
)
target_link_libraries(move
    ${catkin_LIBRARIES}
)

