/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
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
