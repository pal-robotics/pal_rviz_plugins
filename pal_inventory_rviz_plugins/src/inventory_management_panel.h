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

