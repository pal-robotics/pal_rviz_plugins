/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
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

  virtual void onInitialize() override;
};
}

#endif // MAP_MANAGEMENT_PANEL

