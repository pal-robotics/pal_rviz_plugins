/**************************************************************************
**
**  File: navigation_utils_panel.cpp
**
**  Author: victor
**  Created on: 30/08/2016
**
**  Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#include "ui_dock_undock.h"
#include <QMessageBox>
#include <QInputDialog>
#include <pal_navigation_rviz_plugins/dock_undock.h>
#include <ros/ros.h>

namespace pal
{
DockUndockPanel::DockUndockPanel(QWidget *parent) :
  rviz::Panel(parent),
  ui(new Ui::DockUndockPanel),
  _undock_ac("/undocker_server", false),
  _dock_ac("/go_and_dock", false)
{
  ui->setupUi(this);
  connect(ui->dock, SIGNAL(clicked()), this, SLOT(Dock()));
  connect(ui->undock, SIGNAL(clicked()), this, SLOT(Undock()));
  connect(ui->cancelDockUndock, SIGNAL(clicked()),
          this, SLOT(cancelDockUndock()));
}

DockUndockPanel::~DockUndockPanel()
{
  delete ui;
}

void DockUndockPanel::Undock()
{
  laser_servoing_msgs::UndockGoal goal;
  _undock_ac.sendGoal(goal,
               boost::bind(&DockUndockPanel::undockGoalDone, this, _1, _2),
               boost::bind(&DockUndockPanel::undockGoalActive, this));
}

void DockUndockPanel::Dock()
{
  dock_charge_sm_msgs::GoAndDockGoal goal;
  goal.use_current_pose = true;
  _dock_ac.sendGoal(goal, boost::bind(&DockUndockPanel::dockGoalActive, this));
//               boost::bind(&DockUndockPanel::dockGoalDone, this, _1, _2),
//               boost::bind(&DockUndockPanel::dockGoalActive, this));
//  _dock_ac.sendGoal(goal);
}

void DockUndockPanel::cancelDockUndock()
{
  _undock_ac.cancelAllGoals();
  _dock_ac.cancelAllGoals();
}


void DockUndockPanel::undockGoalActive()
{
  ui->feedbackText->setText(QString::fromStdString(
                              _undock_ac.getState().toString()));
  ui->cancelDockUndock->setEnabled(true);
}

void DockUndockPanel::dockGoalActive()
{
  ui->feedbackText->setText(QString::fromStdString(
                              _dock_ac.getState().toString()));
  ui->cancelDockUndock->setEnabled(true);
}

void DockUndockPanel::undockGoalDone(
    const actionlib::SimpleClientGoalState &state,
    const laser_servoing_msgs::UndockResultConstPtr &result)
{
  ui->feedbackText->setText(QString::fromStdString(state.toString()));
  ui->cancelDockUndock->setEnabled(false);
}

void DockUndockPanel::dockGoalDone(
    const actionlib::SimpleClientGoalState &state,
    const dock_charge_sm_msgs::GoAndDockActionResultConstPtr &result)
{
  ui->feedbackText->setText(QString::fromStdString(state.toString()));
  ui->cancelDockUndock->setEnabled(false);
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::DockUndockPanel, rviz::Panel )
