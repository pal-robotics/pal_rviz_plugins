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



private:
  Ui::MapManagementWidgetUI *ui;
  ros::NodeHandle _nh;
  ros::ServiceClient _client;
};

} // pal

#endif // MAP_MANAGEMENT_WIDGET_H
