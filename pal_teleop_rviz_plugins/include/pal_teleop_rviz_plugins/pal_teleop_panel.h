/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef PAL_TELEOP_PANEL_H
#define PAL_TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include <qlistwidget.h>
#include <qlabel.h>
#include <QTouchEvent>
#include <QTimer>
class QLineEdit;


namespace pal
{

namespace Ui {
class PalTeleopPanel;
}
class PalJoystick;
class PalTeleopPanel: public rviz::Panel
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
  PalTeleopPanel( QWidget* parent = 0 );

  ~PalTeleopPanel();

  virtual int heightForWidth( int w );


public Q_SLOTS:
  void joystickPos(qreal x, qreal y);
protected:
  ros::NodeHandle _nh;
  Ui::PalTeleopPanel *ui;
  ros::Publisher _pub;
  PalJoystick *_joy;
  qreal _lastX;
  qreal _lastZ;
};
} // pal

#endif // PAL_TELEOP_PANEL_H
