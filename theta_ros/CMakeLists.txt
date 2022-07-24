cmake_minimum_required(VERSION 3.0.2)
project(theta_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pluginlib
    nodelet
    # camera_apps_msgs
    # pcl_ros
    # eigen_conversions
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES theta_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(image_separater_node src/image_separater_node.cpp src/image_separater.cpp)
add_dependencies(image_separater_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    image_separater_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_library(image_separater_nodelet src/image_separater.cpp src/image_separater_nodelet.cpp)
target_link_libraries(image_separater_nodelet
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)
