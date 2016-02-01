/*
 *  waypoint_group_panel.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#ifndef WAYPOINT_GROUP_PANEL_H
#define WAYPOINT_GROUP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include <qlistwidget.h>
#include <actionlib/client/simple_action_client.h>
#include <pal_waypoint_msgs/DoWaypointNavigationAction.h>

class QLineEdit;

namespace pal
{

namespace Ui {
class WaypointGroupPanel;
}

class WaypointGroupList : public QListWidget
{
    Q_OBJECT
public:
WaypointGroupList(QWidget *parent);

Q_SIGNALS:
void groupPoisChanged();

// QWidget interface
protected:
virtual void contextMenuEvent(QContextMenuEvent *ev);
virtual void dropEvent(QDropEvent *ev);
//virtual void dragMoveEvent(QDragMoveEvent *ev);
};


// DriveWidget class, and is described there.
class WaypointGroupPanel: public rviz::Panel
{

Q_OBJECT
public:

  WaypointGroupPanel( QWidget* parent = 0 );

  ~WaypointGroupPanel();


public Q_SLOTS:
  void updateParamData();
  void groupChanged(const QString &group);
  void newGroup();
  void delGroup();
  void runGroup();
  void stopGroup();
  void saveActiveGroup();

protected:
  void updateButtonStatus() const;
  void goalActive();
  void goalDone(const actionlib::SimpleClientGoalState& state,
                const pal_waypoint_msgs::DoWaypointNavigationResultConstPtr& result);

  void updatePoData(const std::string &param, QListWidget *list);
  ros::NodeHandle _nh;

  Ui::WaypointGroupPanel *ui;
  std::string _poiParam;
  std::string _podParam;
  std::string _groupParam;
  QString _activeGroup;
  actionlib::SimpleActionClient<pal_waypoint_msgs::DoWaypointNavigationAction> _actionClient;
  bool _goalRunning;


};

} // pal

#endif // WAYPOINT_GROUP_PANEL_H
