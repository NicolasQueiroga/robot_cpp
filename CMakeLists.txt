cmake_minimum_required(VERSION 3.0.2)
project(robot_cpp VERSION 1.0.0)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_vision
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

set(AUXILIAR_INCLUDE_DIR
${CMAKE_CURRENT_SOURCE_DIR}/src/Auxiliar)
add_subdirectory(src/Auxiliar)

include_directories(
# include
  ${AUXILIAR_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} Auxiliar ${OpenCV_LIBRARIES})
ADD_DEPENDENCIES(main ${${PROJECT_NAME}_EXPORTED_TARGETS} Auxiliar ${OpenCV_LIBRARIES} ${catkin_EXPORTED_TARGETS})