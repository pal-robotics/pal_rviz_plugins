/*
 *  surveillance_db_group_panel.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#ifndef SURVEILLANCE_DB_GROUP_PANEL_H
#define SURVEILLANCE_DB_GROUP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include <image_transport/image_transport.h>
#include <surveillance_db/surveillance_db.h>
#include <qlabel.h>
#include <std_msgs/String.h>
class QLineEdit;

namespace pal
{

namespace Ui {
class SurveillanceDbPanel;
}

class QRosImageLabel : public QLabel
{
public:
    QRosImageLabel(QWidget* parent = 0);

    virtual ~QRosImageLabel ();
    void setImage(const sensor_msgs::Image::ConstPtr& msg);

};

class SurveillanceDbPanel: public rviz::Panel
{
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT
public:
    SurveillanceDbPanel( QWidget* parent = 0 );

    ~SurveillanceDbPanel();
    void updateDetectionList();
    void insertCb(const std_msgs::StringConstPtr &s);
public Q_SLOTS:

    void recordSelected(int index);

protected:
    Ui::SurveillanceDbPanel *ui;
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    pal::SurveillanceDB _db;
};

} // pal

#endif // SURVEILLANCE_DB_GROUP_PANEL_H
