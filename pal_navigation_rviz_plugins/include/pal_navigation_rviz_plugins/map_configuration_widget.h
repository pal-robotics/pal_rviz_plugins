/*
 *  map_configuration_widget.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 11/9/2015
 *      Author: victor
 */

#ifndef MAP_CONFIGURATION_WIDGET_H
#define MAP_CONFIGURATION_WIDGET_H

#include <QWidget>
namespace pal
{

namespace Ui {
class map_configuration_widgetUI;
}

class map_configuration_widget : public QWidget
{
  Q_OBJECT

public:
  explicit map_configuration_widget(QWidget *parent = 0);
  ~map_configuration_widget();

public Q_SLOTS:
  void refreshMaps();
  void setActive();
  void downloadMap();
  void uploadMap();
  void deleteMap();
  void renameMap();

private:
  Ui::map_configuration_widgetUI *ui;

  std::string getSelectedMap() const;
};
} //namespace pal
#endif // MAP_CONFIGURATION_WIDGET_H
