cmake_minimum_required(VERSION 2.8.3)
project(pal_waypoint_rviz_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pal_waypoint
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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pal_waypoint_rviz_plugins
#  CATKIN_DEPENDS roscpp rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
                  ${CMAKE_CURRENT_BINARY_DIR} # Auto generated qt headers go here
                  ${catkin_INCLUDE_DIRS}
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

QT4_WRAP_CPP(WAYPOINT_GROUP_PANEL_MOC_FILES test/qros_spinner.h include/pal_waypoint_rviz_plugins/waypoint_group_panel.h)
QT4_WRAP_UI(WAYPOINT_GROUP_PANEL_UIS_H src/waypoint_group_panel.ui)


QT4_WRAP_CPP(ADD_POI_TOOL_MOC_FILES include/pal_waypoint_rviz_plugins/add_poi_tool.h)
## Declare a cpp library
 add_library(waypoint_group_panel src/waypoint_group_panel.cpp
                                  ${WAYPOINT_GROUP_PANEL_MOC_FILES}
                                  ${WAYPOINT_GROUP_PANEL_UIS_H})

# add_dependencies(pal_waypoint_rviz_plugins_node pal_waypoint_rviz_plugins_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(waypoint_group_panel
  ${catkin_LIBRARIES}
)

 add_library(add_poi_tool         src/add_poi_tool.cpp
                                  ${ADD_POI_TOOL_MOC_FILES})

# add_dependencies(pal_waypoint_rviz_plugins_node pal_waypoint_rviz_plugins_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(add_poi_tool
  ${catkin_LIBRARIES}
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

## Mark executables and/or libraries for installation
install(TARGETS waypoint_group_panel add_poi_tool
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    #This is not a unit test, just an executable to visualize the rviz panel
    add_executable(waypoint_group_panel_test test/waypoint_group_panel_test.cpp)
    target_link_libraries(waypoint_group_panel_test waypoint_group_panel)
endif(CATKIN_ENABLE_TESTING)
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pal_waypoint_rviz_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
