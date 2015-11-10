/*
 *  pal_teleop_panel_test.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <pal_navigation_rviz_plugins/map_configuration_widget.h>
#include <qapplication.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_configuration_widge_test");
    ros::console::initialize();

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_STREAM("Online");
    QApplication app(argc, argv);
    //    QWidget window;
    //    window.resize(320, 240);
    pal::map_configuration_widget widget;
    widget.show();
    return app.exec();
}
