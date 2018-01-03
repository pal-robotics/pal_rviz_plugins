/**************************************************************************
**
**  File: navigation_utils_panel.h
**
**  Author: victor
**  Created on: 30/08/2016
**
**  Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef NAVIGATION_UTILS_PANEL_H
#define NAVIGATION_UTILS_PANEL_H

#include <QWidget>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rviz/panel.h>
#include <pal_composite_navigation_msgs/GoToFloorPOIAction.h>
#include <actionlib/client/simple_action_client.h>
#endif
namespace Ui {
class NavigationUtilsPanel;
}

namespace pal
{
class NavigationUtilsPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit NavigationUtilsPanel(QWidget *parent = 0);
  ~NavigationUtilsPanel();

private Q_SLOTS:
  void imgFromScan();
  void savePosePOI();
  void sendNavGoal();
  void cancelNavGoal();
private:
  Ui::NavigationUtilsPanel *ui;
  actionlib::SimpleActionClient<pal_composite_navigation_msgs::GoToFloorPOIAction> _ac;
  void goalDone(const actionlib::SimpleClientGoalState &state,
                const pal_composite_navigation_msgs::GoToFloorPOIResultConstPtr &result);
  void goalActive();
};
} //namespace
#endif // BUILDING_MANAGEMENT_PANEL_H
