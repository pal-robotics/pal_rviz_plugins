/*
 *  surveillance_db_group_panel.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */
#include <pal_surveillance_db_rviz_plugins/surveillance_db_panel.h>
#include "ui_surveillance_db_panel.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>


namespace pal
{


SurveillanceDbPanel::SurveillanceDbPanel( QWidget* parent )
    : rviz::Panel( parent ),
      ui(new Ui::SurveillanceDbPanel)
{
    ui->setupUi(this);
    _sub = _nh.subscribe("/warehouse/surveillance_db/images/inserts", 1, &SurveillanceDbPanel::insertCb, this);
    connect(ui->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(recordSelected(int)));
    updateDetectionList();
}

SurveillanceDbPanel::~SurveillanceDbPanel()
{
    delete ui;
}

void SurveillanceDbPanel::updateDetectionList()
{
    std::map<std::string, ros::Time> records =_db.getRecords();
    ROS_INFO_STREAM("Got " << records.size() << " records");
    ui->comboBox->clear();

    // Reverse loop so most recent records are first
    for (std::map<std::string, ros::Time>::reverse_iterator it = records.rbegin();
         it != records.rend(); ++it)
    {
        ui->comboBox->insertItem(ui->comboBox->count(), QString::fromStdString(it->first));
    }
}


void SurveillanceDbPanel::insertCb(const std_msgs::StringConstPtr &s)
{
    updateDetectionList();
}

void SurveillanceDbPanel::recordSelected(int index)
{
    QString tId = ui->comboBox->itemText(index);

    ROS_INFO_STREAM("Displaying " << tId.toStdString());
    try
    {
        sensor_msgs::Image img = _db.getImgRecord(tId.toStdString());
        sensor_msgs::ImagePtr ptr(new sensor_msgs::Image);
        *ptr = img;
        ui->image_display->setImage(ptr);
    }
    catch (const std::exception &e)
    {
        ui->image_display->setPixmap(QPixmap());
    }
}
} // end namespace pal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::SurveillanceDbPanel, rviz::Panel )
