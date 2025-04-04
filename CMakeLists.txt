cmake_minimum_required(VERSION 3.0.2)
project(ros_jog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
  geometry_msgs
  controller_manager_msgs
  control_msgs
  trajectory_msgs
  taskspace_control_msgs
  eigen_conversions
  actionlib
)

find_package(Eigen3 REQUIRED)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rqt_gui
    rqt_gui_cpp
    std_msgs
    geometry_msgs
    controller_manager_msgs
    control_msgs
    trajectory_msgs
    taskspace_control_msgs
    eigen_conversions
    actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(ros_jog_SRCS
  src/ros_jog.cpp
  src/controller_manager_client.cpp
  src/controller_clients.cpp
)

set(ros_jog_HDRS
  include/ros_jog/ros_jog.h
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(ros_jog_MOCS ${ros_jog_HDRS})
else()
  qt4_wrap_cpp(ros_jog_MOCS ${ros_jog_HDRS})
endif()

add_library(${PROJECT_NAME}
  ${ros_jog_SRCS}
  ${ros_jog_MOCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_subdirectory(jogging_example)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
