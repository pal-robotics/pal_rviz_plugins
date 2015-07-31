/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author: victor
 */

#ifndef MAP_MANAGEMENT_PANEL
#define MAP_MANAGEMENT_PANEL

#include <rviz/panel.h>

namespace pal
{


class MapManagementPanel: public rviz::Panel
{
  Q_OBJECT
public:
  MapManagementPanel( QWidget* parent = 0 );

  ~MapManagementPanel();
};
}

#endif // MAP_MANAGEMENT_PANEL

