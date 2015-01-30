/*
 *  surveillance_db_group_panel_test.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <pal_surveillance_db_rviz_plugins/surveillance_db_panel.h>
#include <qapplication.h>
#include <qtimer.h>
#include "qros_spinner.h"
// Include header files for application components.
// ...


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "surveillance_db_group_panel_test");
    ros::console::initialize();
    ROS_INFO_STREAM("Online");
    QApplication app(argc, argv);
//    QWidget window;
//    window.resize(320, 240);

    pal::SurveillanceDbPanel panel;
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("image", 1, boost::bind(&pal::SurveillanceDbPanel::setImage, &panel, _1));
    panel.show();
    QRosSpinner spinner;

    return app.exec();
}
