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

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_library(aux_opencv SHARED ${CMAKE_CURRENT_SOURCE_DIR}/libs/aux_opencv/aux_opencv.cpp)
add_library(robot SHARED ${CMAKE_CURRENT_SOURCE_DIR}/libs/robot/robot.cpp)
add_library(actions SHARED ${CMAKE_CURRENT_SOURCE_DIR}/libs/robot/actions.cpp)
add_executable(run ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

target_include_directories(aux_opencv PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
target_include_directories(actions PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

target_link_libraries(aux_opencv PUBLIC ${OpenCV_LIBRARIES})
target_link_libraries(robot PUBLIC aux_opencv ${catkin_LIBRARIES})
target_link_libraries(actions PUBLIC aux_opencv robot ${catkin_LIBRARIES})
target_link_libraries(run PUBLIC robot actions)