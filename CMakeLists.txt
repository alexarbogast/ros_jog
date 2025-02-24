cmake_minimum_required(VERSION 3.0.2)
project(ros_jog)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  taskspace_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    taskspace_control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(console_jog src/console_jog.cpp)
target_link_libraries(console_jog ${catkin_LIBRARIES})

qt5_wrap_cpp(MOC 
  include/ros_jog/jogging_window.h
  include/ros_jog/cartesian_jog_widget.h
)

add_executable(jogging_window 
  src/jogging_window.cpp 
  src/cartesian_jog_widget.cpp 
  ${MOC}
)
target_link_libraries(jogging_window ${catkin_LIBRARIES} Qt5::Widgets)

add_subdirectory(jogging_example)

## Installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
