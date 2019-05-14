/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
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
