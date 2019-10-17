#ifndef INVENTORY_GROUP_PANEL_H
#define INVENTORY_GROUP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include <pal_inventory_msgs/ActiveInventory.h>
#include <QObject>
class QLineEdit;

namespace pal
{

namespace Ui {
class ZOIInventoryPanel;
}

/**
 * @brief The InventoryZonePanel class controls a simple panel with 3 options:
 *          - Show all ZOIs
 *          - Show active ZOIs
 *          - Hide all ZOIs
 *
 *        Due to the nature of rviz, we cannot exclusively modify what
 *        is displayed in the instance of rviz where the panel is, therefore
 *        what this Panel does is configure a regex that filters the displayed
 *        Zois and sends the regex to the ZOI marker server
 */
class InventoryZonePanel: public rviz::Panel
{
Q_OBJECT
public:
  InventoryZonePanel( QWidget* parent = 0 );

  ~InventoryZonePanel();

  void activeInventoryCb(const pal_inventory_msgs::ActiveInventory &msg);

public Q_SLOTS:
  void modeChanged();

private:
  void apply();
  enum ZoneDisplayMode
  {
    SHOW_ALL,
    SHOW_ACTIVE,
    HIDE_ALL
  };

  ros::NodeHandle _nh;
  Ui::ZOIInventoryPanel *ui;
  ros::Subscriber _sub;
  ros::Publisher _pub;
  ZoneDisplayMode _mode;
  pal_inventory_msgs::ActiveInventory _msg;
};

} // pal

#endif // INVENTORY_GROUP_PANEL_H
