#include "inventory_management_panel.h"
#include <pal_inventory_rviz_plugins/inventory_configuration_widget.h>
#include <QHBoxLayout>
namespace pal
{

InventoryManagementPanel::InventoryManagementPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(new inventory_configuration_widget(this));
}

InventoryManagementPanel::~InventoryManagementPanel()
{
}

} // end namespace pal


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::InventoryManagementPanel, rviz::Panel )
