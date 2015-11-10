/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author: victor
 */
#include "map_management_panel.h"
#include <pal_navigation_rviz_plugins/map_management_widget.h>
#include <pal_navigation_rviz_plugins/map_configuration_widget.h>
#include <QHBoxLayout>
namespace pal
{

MapManagementPanel::MapManagementPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(new map_configuration_widget(this));
  layout->addWidget(new MapManagementWidget(this));
}

MapManagementPanel::~MapManagementPanel()
{
}

} // end namespace pal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::MapManagementPanel, rviz::Panel )
