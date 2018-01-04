cmake_minimum_required(VERSION 2.8.3)
project(pal_inventory_rviz_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pal_inventory_msgs
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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pal_inventory_rviz_plugins
#  CATKIN_DEPENDS roscpp rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR} # Auto generated qt headers go here
                           ${catkin_INCLUDE_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Declare a cpp library
#Headers are needed by AUTOMOC, otherwise we get linking errors
 add_library(inventory_configuration_widget src/inventory_configuration_widget.cpp
                                            include/pal_inventory_rviz_plugins/inventory_configuration_widget.h)

## Specify libraries to link a library or executable target against
target_link_libraries(inventory_configuration_widget
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_library(inventory_management_panel src/inventory_management_panel.cpp
                                        src/inventory_management_panel.h)

 add_library(inventory_zone_panel src/inventory_zone_panel.cpp
                                  include/pal_inventory_rviz_plugins/inventory_zone_panel.h)

# add_dependencies(pal_inventory_rviz_plugins_node pal_inventory_rviz_plugins_generate_messages_cpp)



target_link_libraries(inventory_management_panel
  ${QT_LIBRARIES}
  inventory_configuration_widget
)

target_link_libraries(inventory_zone_panel
  ${QT_LIBRARIES}
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
install(TARGETS inventory_zone_panel inventory_management_panel inventory_configuration_widget
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#install(DIRECTORY icons
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

