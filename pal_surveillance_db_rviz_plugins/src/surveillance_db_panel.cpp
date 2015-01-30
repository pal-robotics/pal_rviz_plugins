/*
 *  surveillance_db_group_panel.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */
#include "ui_surveillance_db_panel.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <pal_surveillance_db_rviz_plugins/surveillance_db_panel.h>
#include <image_transport/image_transport.h>


#include <rqt_image_view/ratio_layouted_frame.h>
#include <ros/console.h>


namespace pal
{

QRosImageLabel::QRosImageLabel(QWidget *parent)
    : QLabel(parent)
{
}

QRosImageLabel::~QRosImageLabel()
{
}

void QRosImageLabel::setImage(const sensor_msgs::Image::ConstPtr& msg)
{
    cv::Mat conversion_mat_;
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (msg->encoding == "CV_8UC3")
            {
                // assuming it is rgb
                conversion_mat_ = cv_ptr->image;
            } else if (msg->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
            } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
                // scale / quantify
                double min = 0;
                double max = 10; //ui->max_range_double_spin_box->value();
                if (msg->encoding == "16UC1") max *= 1000;
                //                if (ui->dynamic_range_check_box->isChecked())
                //                {
                //                    // dynamically adjust range based on min/max in image
                //                    cv::minMaxLoc(cv_ptr->image, &min, &max);
                //                    if (min == max) {
                //                        // completely homogeneous images are displayed in gray
                //                        min = 0;
                //                        max = 2;
                //                    }
                //                }
                cv::Mat img_scaled_8u;
                cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
                cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
            } else {
                ROS_WARN("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
                setPixmap(QPixmap());
                return;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
            setPixmap(QPixmap());
            return;
        }
    }
    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    setPixmap(QPixmap::fromImage(image));
    //    if (!ui->zoom_1_push_button->isEnabled())
    //    {
    //        ui->zoom_1_push_button->setEnabled(true);
    //        onZoom1(ui->zoom_1_push_button->isChecked());
    //    }
}


SurveillanceDbPanel::SurveillanceDbPanel( QWidget* parent )
    : rviz::Panel( parent ),
      ui(new Ui::SurveillanceDbPanel)
{
    ui->setupUi(this);
    _sub = _nh.subscribe("/warehouse/surveillance_db/images/inserts", 1, &SurveillanceDbPanel::insertCb, this);
    connect(ui->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(recordSelected(int)));
}

SurveillanceDbPanel::~SurveillanceDbPanel()
{
    delete ui;
}


void SurveillanceDbPanel::insertCb(const std_msgs::StringConstPtr &s)
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

void SurveillanceDbPanel::recordSelected(int index)
{
    QString tId = ui->comboBox->itemText(index);

    ROS_INFO_STREAM("DIsplaying " << tId.toStdString());
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
