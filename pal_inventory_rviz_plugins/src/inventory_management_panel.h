/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef INVENTORY_MANAGEMENT_PANEL
#define INVENTORY_MANAGEMENT_PANEL

#include <rviz/panel.h>

namespace pal
{


class InventoryManagementPanel: public rviz::Panel
{
  Q_OBJECT
public:
  InventoryManagementPanel( QWidget* parent = 0 );

  ~InventoryManagementPanel();
};
}

#endif // INVENTORY_MANAGEMENT_PANEL

