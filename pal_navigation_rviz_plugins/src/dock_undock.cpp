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
DockUndockPanel::DockUndockPanel(QWidget *parent)
  : rviz::Panel(parent)
  , ui(new Ui::DockUndockPanel)
  , undock_ac_("/undocker_server", false)
  , dock_ac_("/go_and_dock", false)
  , create_dock_ac_("/create_dockstation", false)
{
  ui->setupUi(this);
  connect(ui->dock, SIGNAL(clicked()), this, SLOT(Dock()));
  connect(ui->undock, SIGNAL(clicked()), this, SLOT(Undock()));
  connect(ui->createDock, SIGNAL(clicked()), this, SLOT(createDock()));
  connect(ui->cancelDockUndock, SIGNAL(clicked()), this, SLOT(cancelDockUndock()));
}

DockUndockPanel::~DockUndockPanel()
{
  delete ui;
}


void DockUndockPanel::Undock()
{
  laser_servoing_msgs::UndockGoal goal;
  undock_ac_.sendGoal(goal, boost::bind(&DockUndockPanel::undockGoalDone, this, _1, _2),
                      boost::bind(&DockUndockPanel::undockGoalActive, this));
}

void DockUndockPanel::Dock()
{
  dock_charge_sm_msgs::GoAndDockGoal goal;
  goal.use_current_pose = true;
  dock_ac_.sendGoal(goal, boost::bind(&DockUndockPanel::dockGoalDone, this, _1, _2),
                    boost::bind(&DockUndockPanel::dockGoalActive, this));
}

void DockUndockPanel::createDock()
{
  pal_common_msgs::EmptyGoal goal;
  create_dock_ac_.sendGoal(goal, boost::bind(&DockUndockPanel::createDockGoalDone, this, _1, _2),
                           boost::bind(&DockUndockPanel::createDockGoalActive, this));
}

void DockUndockPanel::cancelDockUndock()
{
  undock_ac_.cancelAllGoals();
  dock_ac_.cancelAllGoals();
  create_dock_ac_.cancelAllGoals();
}

void DockUndockPanel::setActionButtons(const bool enable)
{
  if (enable)
  {
    ui->cancelDockUndock->setEnabled(false);
    ui->dock->setEnabled(true);
    ui->undock->setEnabled(true);
    ui->createDock->setEnabled(true);
  }
  else
  {
    ui->cancelDockUndock->setEnabled(true);
    ui->dock->setEnabled(false);
    ui->undock->setEnabled(false);
    ui->createDock->setEnabled(false);
  }
}

void DockUndockPanel::undockGoalActive()
{
  ui->feedbackText->setText(QString::fromStdString(undock_ac_.getState().toString()));
  setActionButtons(false);
}

void DockUndockPanel::dockGoalActive()
{
  ui->feedbackText->setText(QString::fromStdString(dock_ac_.getState().toString()));
  setActionButtons(false);
}

void DockUndockPanel::createDockGoalActive()
{
  ui->feedbackText->setText(QString::fromStdString(create_dock_ac_.getState().toString()));
  setActionButtons(false);
}

void DockUndockPanel::undockGoalDone(const actionlib::SimpleClientGoalState &state,
                                     const laser_servoing_msgs::UndockResultConstPtr &result)
{
  ui->feedbackText->setText(QString::fromStdString(state.toString()));
  setActionButtons(true);
}

void DockUndockPanel::dockGoalDone(const actionlib::SimpleClientGoalState &state,
                                   const dock_charge_sm_msgs::GoAndDockResultConstPtr &result)
{
  ui->feedbackText->setText(QString::fromStdString(state.toString()));
  setActionButtons(true);
}

void DockUndockPanel::createDockGoalDone(const actionlib::SimpleClientGoalState &state,
                                         const pal_common_msgs::EmptyResultConstPtr &result)
{
  ui->feedbackText->setText(QString::fromStdString(state.toString()));
  setActionButtons(true);
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::DockUndockPanel, rviz::Panel)
