/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
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
