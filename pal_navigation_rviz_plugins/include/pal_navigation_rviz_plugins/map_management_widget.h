/*
 *  map_management_widget.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#ifndef MAP_MANAGEMENT_WIDGET_H
#define MAP_MANAGEMENT_WIDGET_H

#include <ros/ros.h>
#include <qwidget.h>
#include <pal_navigation_msgs/NavigationStatus.h>
namespace pal
{

namespace Ui {
class MapManagementWidgetUI;
}

class MapManagementWidget : public QWidget
{
  Q_OBJECT
public:
  MapManagementWidget( QWidget* parent = 0 );

  ~MapManagementWidget();

public Q_SLOTS:
  void saveMapConfig();
  void startMapping();
  void stopMapping();



private:
  void smStateCb(const pal_navigation_msgs::NavigationStatusConstPtr &msg);

  Ui::MapManagementWidgetUI *ui_;
  ros::NodeHandle nh_;
  ros::ServiceClient poi_changed_client_;
  ros::ServiceClient navigation_sm_client_;
  ros::Subscriber navigation_sm_state_sub_;
};

} // pal

#endif // MAP_MANAGEMENT_WIDGET_H
