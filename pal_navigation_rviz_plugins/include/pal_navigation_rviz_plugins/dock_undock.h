/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
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

namespace Ui
{
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
  actionlib::SimpleActionClient<laser_servoing_msgs::UndockAction> undock_ac_;
  actionlib::SimpleActionClient<dock_charge_sm_msgs::GoAndDockAction> dock_ac_;
  actionlib::SimpleActionClient<pal_common_msgs::EmptyAction> create_dock_ac_;

  void setActionButtons(const bool enable);

  void undockGoalActive();

  void dockGoalActive();

  void createDockGoalActive();

  void undockGoalDone(const actionlib::SimpleClientGoalState &state,
                      const laser_servoing_msgs::UndockResultConstPtr &result);

  void dockGoalDone(const actionlib::SimpleClientGoalState &state,
                    const dock_charge_sm_msgs::GoAndDockResultConstPtr &result);

  void createDockGoalDone(const actionlib::SimpleClientGoalState &state,
                          const pal_common_msgs::EmptyResultConstPtr &result);
};
}  // namespace
#endif  // DOCK_UNDOCK_PANEL_H_PANEL_H
