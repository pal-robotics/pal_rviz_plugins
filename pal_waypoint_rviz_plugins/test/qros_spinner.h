/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef QROSSPINNER_H
#define QROSSPINNER_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <qobject.h>
#include <qtimer.h>
#include <ros/ros.h>
#endif

// This class configures a Qt Timer to trigger ros spinOnces
// Useful for executing ROS callbacks in the qt thread
class QRosSpinner : public QObject
{
    Q_OBJECT
public:
    QRosSpinner();
    virtual ~QRosSpinner();

    public Q_SLOTS:
    void doSpin();
private:
    QTimer tim;

};


#endif // QROSSPINNER_H

