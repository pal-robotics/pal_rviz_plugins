/*
 *  pal_teleop_panel_test.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <pal_teleop_rviz_plugins/pal_teleop_panel.h>
#include <qapplication.h>
// Include header files for application components.
// ...

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_group_panel_test");
    ros::console::initialize();
    ROS_INFO_STREAM("Online");
    QApplication app(argc, argv);
    //    QWidget window;
    //    window.resize(320, 240);
    pal::PalTeleopPanel panel;
    panel.show();
    return app.exec();
}
