/**************************************************************************
**
**  File: navigation_utils_panel.h
**
**  Author: victor
**  Created on: 30/08/2016
**
**  Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef DOCK_UNDOCK_PANEL_H
#define DOCK_UNDOCK_PANEL_H

#include <QWidget>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rviz/panel.h>
#include <laser_servoing_msgs/UndockAction.h>
#include <dock_charge_sm_msgs/GoAndDockAction.h>
#include <pal_common_msgs/EmptyAction.h>

#include <actionlib/client/simple_action_client.h>
#endif

namespace Ui {
class DockUndockPanel;
}

namespace pal
{
class DockUndockPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit DockUndockPanel(QWidget *parent = 0);
  ~DockUndockPanel();

private Q_SLOTS:
  void Dock();
  void Undock();
  void createDock();
  void cancelDockUndock();
private:
  Ui::DockUndockPanel *ui;
  actionlib::SimpleActionClient<laser_servoing_msgs::UndockAction> _undock_ac;
  actionlib::SimpleActionClient<dock_charge_sm_msgs::GoAndDockAction> _dock_ac;
  actionlib::SimpleActionClient<pal_common_msgs::EmptyAction> create_dock_ac_;

  void undockGoalDone(
      const actionlib::SimpleClientGoalState &state,
      const laser_servoing_msgs::UndockResultConstPtr &result);
  void undockGoalActive();

  void dockGoalDone(const actionlib::SimpleClientGoalState &state,
                    const dock_charge_sm_msgs::GoAndDockResultConstPtr &result);
  void dockGoalActive();

  void createDockGoalDone(const actionlib::SimpleClientGoalState &state,
                    const pal_common_msgs::EmptyResultConstPtr &result);
  void createDockGoalActive();

  void setActionButtons(const bool enable);
};
} //namespace
#endif // DOCK_UNDOCK_PANEL_H_PANEL_H
