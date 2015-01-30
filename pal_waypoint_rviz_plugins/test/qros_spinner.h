/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 1/21/2015
 *      Author: victor
 */

#ifndef QROSSPINNER_H
#define QROSSPINNER_H

#include <qobject.h>
#include <qtimer.h>
#include <ros/ros.h>

// This class configures a Qt Timer to trigger ros spinOnces
// Useful for executing ROS callbacks in the qt thread
class QRosSpinner : public QObject
{
    Q_OBJECT
public:
    QRosSpinner()
     : QObject()
    {
        tim.setInterval(100);
        connect(&tim, SIGNAL(timeout()), this, SLOT(doSpin()));
        tim.start();
    }
    virtual ~QRosSpinner()
    {

    }

    public Q_SLOTS:
    void doSpin()
    {
        ros::spinOnce();
    }
private:
    QTimer tim;

};


#endif // QROSSPINNER_H
