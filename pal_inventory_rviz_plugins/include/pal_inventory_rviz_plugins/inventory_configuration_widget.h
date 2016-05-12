/*
 *  inventory_configuration_widget.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 11/9/2015
 *      Author: victor
 */

#ifndef INVENTORY_CONFIGURATION_WIDGET_H
#define INVENTORY_CONFIGURATION_WIDGET_H

#include <QWidget>
namespace pal
{

namespace Ui {
class inventory_configuration_widgetUI;
}

class inventory_configuration_widget : public QWidget
{
  Q_OBJECT

public:
  explicit inventory_configuration_widget(QWidget *parent = 0);
  ~inventory_configuration_widget();

public Q_SLOTS:
  void refreshInventories();
  void downloadInventory();
  void deleteInventory();
  void renameInventory();

private:
  Ui::inventory_configuration_widgetUI *ui;

  std::string getSelectedInventory() const;
};
} //namespace pal
#endif // INVENTORY_CONFIGURATION_WIDGET_H
