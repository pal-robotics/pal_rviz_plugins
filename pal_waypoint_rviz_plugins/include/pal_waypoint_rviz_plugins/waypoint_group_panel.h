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

class QLineEdit;

namespace pal
{

namespace Ui {
class WaypointGroupPanel;
}

//class WaypointListItem : public QListWidgetItem
//{
//public:
//    WaypointListItem(QListWidget *view, const QString &id, const QString &name);

//    QString _id;
//    QString _name;
//};

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

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class WaypointGroupPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  WaypointGroupPanel( QWidget* parent = 0 );

  ~WaypointGroupPanel();


public Q_SLOTS:
  void updatePoiList();
  void groupChanged(const QString &group);
  void newGroup();
  void delGroup();
  void saveActiveGroup();

protected:
  ros::NodeHandle _nh;

  Ui::WaypointGroupPanel *ui;
  std::string _poiParam;
  std::string _groupParam;
  QString _activeGroup;

};

} // pal

#endif // WAYPOINT_GROUP_PANEL_H
