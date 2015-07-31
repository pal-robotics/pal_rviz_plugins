/*
 *  map_management_widget.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <pal_navigation_rviz_plugins/map_management_widget.h>
#include "ui_map_management_widget.h"
#include <std_srvs/Empty.h>

namespace pal
{
MapManagementWidget::MapManagementWidget( QWidget* parent )
  :QWidget(parent), ui(new Ui::MapManagementWidgetUI)
{
  ui->setupUi(this);
  connect(ui->save_map_config, SIGNAL(clicked()),this, SLOT(saveMapConfig()));

  // Not persistent because we're not calling this often
  _client = _nh.serviceClient<std_srvs::Empty>("/notifyPoisChanged", false);
}

MapManagementWidget::~MapManagementWidget()
{
  delete ui;
}

void MapManagementWidget::saveMapConfig()
{
  std_srvs::Empty srv;
  ui->status_label->setText("");
  if (!_client.call(srv))
  {
    ui->status_label->setText("Error!");
    ROS_ERROR_STREAM("Error saving map configuration");
  }
  else
  {
    ui->status_label->setText("Ok");
    ROS_INFO_STREAM("Map configuration saved on the robot");
  }
}

} // end namespace pal
