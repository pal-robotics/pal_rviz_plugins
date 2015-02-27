/*
 *  inventory_zone_panel.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <pal_inventory_rviz_plugins/inventory_zone_panel.h>
#include "ui_inventory_zone_panel.h"
#include <std_msgs/String.h>

namespace pal
{
InventoryZonePanel::InventoryZonePanel( QWidget* parent )
  : rviz::Panel( parent ),
    ui(new Ui::ZOIInventoryPanel)
{
  ui->setupUi(this);
  connect(ui->allZones, SIGNAL(toggled(bool)),this, SLOT(modeChanged()));
  connect(ui->activeZones, SIGNAL(toggled(bool)),this, SLOT(modeChanged()));
  connect(ui->hideZones, SIGNAL(toggled(bool)),this, SLOT(modeChanged()));

  _sub = _nh.subscribe("/inventory/active_inventory", 1,
                       &InventoryZonePanel::activeInventoryCb, this);
  _pub = _nh.advertise<std_msgs::String>("/zone_marker_server/regex", 1, true);

  ui->allZones->setChecked(true);
  apply();
}

InventoryZonePanel::~InventoryZonePanel()
{
  delete ui;
}

void InventoryZonePanel::activeInventoryCb(const pal_inventory_msgs::ActiveInventory &msg)
{
  _msg = msg;
  apply();
}

void InventoryZonePanel::modeChanged()
{
  if (ui->allZones->isChecked())
    _mode = SHOW_ALL;
  else if (ui->activeZones->isChecked())
    _mode = SHOW_ACTIVE;
  else if (ui->hideZones->isChecked())
    _mode = HIDE_ALL;

  apply();
}

void InventoryZonePanel::apply()
{
  std::string regex;
  switch (_mode) {
    case SHOW_ACTIVE:
      // The regex will have the format "^(ZONEA|ZONEB)$"
      // It will only match ZONEA or ZONEB
      for (int i = 0; i < _msg.valid_zois.size(); ++i)
      {
        std::string zoi = _msg.valid_zois[i];
        if (regex.empty())
          regex = "^" + zoi;
        else
          regex += "|" + zoi;
      }
      if (!regex.empty())
        regex += "$";
      break;
    case HIDE_ALL:
      regex = ""; // match nothing
      break;
    case SHOW_ALL:
    default:
      regex = ".*"; // match all
      break;
  }
  std_msgs::String s;
  s.data = regex;
  _pub.publish(s);
}


} // end namespace pal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::InventoryZonePanel, rviz::Panel )
