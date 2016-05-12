cmake_minimum_required(VERSION 2.8.3)
project(pal_navigation_rviz_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_srvs
  pal_navigation_msgs
  pluginlib
  roscpp
  rviz
  tf
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
 INCLUDE_DIRS include
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
include_directories(include
                  ${CMAKE_CURRENT_BINARY_DIR} # Auto generated qt headers go here
                  ${catkin_INCLUDE_DIRS}
)


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

QT4_WRAP_CPP(MAP_CONFIGURATION_WIDGET_MOC_FILES include/pal_navigation_rviz_plugins/map_configuration_widget.h)
#QT4_WRAP_CPP(MAP_CONFIGURATION_PANEL_MOC_FILES src/map_configuration_panel.h)
QT4_WRAP_UI(MAP_CONFIGURATION_WIDGET_UIS_H src/map_configuration_widget.ui)

QT4_WRAP_CPP(MAP_MANAGEMENT_WIDGET_MOC_FILES include/pal_navigation_rviz_plugins/map_management_widget.h)
QT4_WRAP_CPP(MAP_MANAGEMENT_PANEL_MOC_FILES src/map_management_panel.h)
QT4_WRAP_UI(MAP_MANAGEMENT_WIDGET_UIS_H src/map_management_widget.ui)


## Declare a cpp library
 add_library(map_configuration_widget src/map_configuration_widget.cpp
                                  ${MAP_CONFIGURATION_WIDGET_MOC_FILES}
                                  ${MAP_CONFIGURATION_WIDGET_UIS_H})

 add_library(map_management_widget src/map_management_widget.cpp
                                  ${MAP_MANAGEMENT_WIDGET_MOC_FILES}
                                  ${MAP_MANAGEMENT_WIDGET_UIS_H})


 add_library(map_management_panel src/map_management_panel.cpp
                                  ${MAP_MANAGEMENT_PANEL_MOC_FILES})

# add_dependencies(pal_navigation_rviz_plugins_node pal_navigation_rviz_plugins_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(map_management_widget
  ${catkin_LIBRARIES}
)

target_link_libraries(map_configuration_widget
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

target_link_libraries(map_management_panel
  map_management_widget
  map_configuration_widget
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark executables and/or libraries for installation
install(TARGETS map_management_widget map_management_panel map_configuration_widget
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#install(DIRECTORY icons
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    # This is not an unit test, just an executable to visualize the rviz panel
    add_executable(map_configuration_widget_test test/map_configuration_widget_test.cpp)
    target_link_libraries(map_configuration_widget_test map_configuration_widget)
endif(CATKIN_ENABLE_TESTING)
