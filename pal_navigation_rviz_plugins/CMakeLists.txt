cmake_minimum_required(VERSION 2.8.3)
project(pal_navigation_rviz_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_srvs
  pal_navigation_msgs
  laser_pattern_detector_msgs
  pal_composite_navigation_msgs
  dock_charge_sm_msgs
  laser_servoing_msgs
  pluginlib
  roscpp
  roslib
  tf
  rviz
  cv_bridge
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

#Required for catkin_package
set(QT_INCLUDE_DIRS ${QT_INCLUDE_DIR}
                    ${QT_QTCORE_INCLUDE_DIR}
                    ${QT_QTGUI_INCLUDE_DIR})

catkin_package(
 LIBRARIES map_management_widget 
 CATKIN_DEPENDS roscpp
 DEPENDS QT
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(SYSTEM
                  ${CMAKE_CURRENT_BINARY_DIR} # Auto generated qt headers go here
                  ${catkin_INCLUDE_DIRS}
)


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(NAVIGATION_WIDGETS
    map_configuration_widget
    map_management_widget
    navigation_utils_panel
    dock_undock)
foreach(widget ${NAVIGATION_WIDGETS})
    QT4_WRAP_CPP(${widget}_MOC_FILES include/${PROJECT_NAME}/${widget}.h)
    QT4_WRAP_UI(${widget}_UIS_H src/${widget}.ui)
    add_library(${widget} src/${widget}.cpp ${${widget}_MOC_FILES} ${${widget}_UIS_H})
    target_link_libraries(${widget}  ${catkin_LIBRARIES} ${QT_LIBRARIES})
endforeach()

QT4_WRAP_CPP(MAP_MANAGEMENT_PANEL_MOC_FILES src/map_management_panel.h)


add_library(map_management_panel src/map_management_panel.cpp
                                  ${MAP_MANAGEMENT_PANEL_MOC_FILES})

target_link_libraries(map_management_panel
  map_management_widget
  map_configuration_widget
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${NAVIGATION_WIDGETS} map_management_panel
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    # This is not an unit test, just an executable to visualize the rviz panel
    add_executable(map_configuration_widget_test test/map_configuration_widget_test.cpp)
    target_link_libraries(map_configuration_widget_test map_configuration_widget)
endif(CATKIN_ENABLE_TESTING)
