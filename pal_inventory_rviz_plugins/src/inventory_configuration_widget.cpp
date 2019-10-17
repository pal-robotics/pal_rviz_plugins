#include <pal_inventory_rviz_plugins/inventory_configuration_widget.h>
#include "ui_inventory_configuration_widget.h"
#include <qmessagebox.h>
#include <qinputdialog.h>
#include <qfiledialog.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <iostream>
#include <stdio.h>


namespace pal
{

std::string getInventoryDir()
{
  std::string host = ros::master::getHost();
  if (host == "localhost")
    return "~/inventory/";
  else
    return "\\$HOME/inventory/";
}

/**
 * @brief exec Execute command and return std output
 */
std::string exec(const std::string &cmd) {
  ROS_DEBUG_STREAM("Executing: " << cmd);
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while (!feof(pipe)) {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);
  ROS_DEBUG_STREAM("Output: \n" << result);
  return result;
}

/**
 * @brief exec_remote_cmd Executes command on current ROS_MASTER_URI host.
 *        If not localhost, will execute it with prepending "ssh pal@hostname "
 *        Does not check that the command can be properly executed remotely
 * @param cmd command to be executed
 * @return output of the command
 */
std::string exec_remote_cmd(const std::string &cmd)
{
  std::string host = ros::master::getHost();
  if (host == "localhost")
    return exec(cmd);
  else
  {
    std::string remote_cmd = "ssh pal@" + host + " " + cmd;
    return exec(remote_cmd);
  }
}

inventory_configuration_widget::inventory_configuration_widget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::inventory_configuration_widgetUI)
{
  ui->setupUi(this);
  connect(ui->refresh_inventorys, SIGNAL(clicked()), this, SLOT(refreshInventories()));
  connect(ui->download_inventory, SIGNAL(clicked()), this, SLOT(downloadInventory()));
  connect(ui->delete_inventory, SIGNAL(clicked()), this, SLOT(deleteInventory()));
  connect(ui->rename_inventory, SIGNAL(clicked()), this, SLOT(renameInventory()));

  std::string host = ros::master::getHost();
  ui->label->setText(QString::fromStdString(host + " inventories:"));
}

inventory_configuration_widget::~inventory_configuration_widget()
{
  delete ui;
}

void inventory_configuration_widget::refreshInventories()
{
  ui->listWidget->clear();
  std::string cmd =
      "ls " + getInventoryDir() + "| xargs -l1 basename";

  std::string output = exec_remote_cmd(cmd);

  std::vector<std::string> inventorys;
  boost::split(inventorys, output, boost::is_any_of("\n"));

  for(auto s : inventorys)
  {
    if (s == "")
      continue;
    QString inventory = QString::fromStdString(s);
    QListWidgetItem *item = new QListWidgetItem(inventory, ui->listWidget);
    ui->listWidget->addItem(item);
  }

  if (!ui->listWidget->currentItem())
    ui->listWidget->setCurrentRow(0);

  bool enable = !inventorys.empty();
  ui->download_inventory->setEnabled(enable);
  ui->delete_inventory->setEnabled(enable);
  ui->rename_inventory->setEnabled(enable);
}

void inventory_configuration_widget::downloadInventory()
{
  std::string inventory = getSelectedInventory();
  std::string host = ros::master::getHost();

  QString target_dir = QFileDialog::getExistingDirectory(this, "Select directory to save inventory to",
                                                         "/home", QFileDialog::ShowDirsOnly);

  if (target_dir.isNull())
    return;

  std::string cmd;
  if (host == "localhost")
    cmd = "cp -r " + getInventoryDir() + inventory + " " +
          target_dir.toStdString();
  else
    cmd = "scp -r pal@" + host + ":" + getInventoryDir() + inventory + " " +
          target_dir.toStdString();
  exec(cmd);
}

void inventory_configuration_widget::deleteInventory()
{
  std::string inventory = getSelectedInventory();
  if (inventory == "")
    return;
  if (QMessageBox::question(this, "Delete inventory",
                            "Delete inventory \"" + QString::fromStdString(inventory) + "\"?",
                            QMessageBox::StandardButton::Ok | QMessageBox::StandardButton::Cancel)
      != QMessageBox::StandardButton::Ok)
    return;

  std::string cmd = "rm -rf " + getInventoryDir() + inventory;
  exec_remote_cmd(cmd);
  refreshInventories();
}

void inventory_configuration_widget::renameInventory()
{
  std::string inventory = getSelectedInventory();

  QRegExp regexp("[A-Z0-9_\-]+",Qt::CaseInsensitive);
  QString name = QString::fromStdString(inventory);
  bool ok = false;
  do
  {
    name = QInputDialog::getText(this, "Enter new inventory name",
                                 QString::fromStdString("Please enter new name for inventory " + inventory +
                                                        " (only letters, numbers, - and _ )"),
                                 QLineEdit::EchoMode::Normal, name, &ok);
  }
  while (ok && !regexp.exactMatch(name));

  std::string cmd = "mv " + getInventoryDir()  + inventory + " " +
                    getInventoryDir() + name.toStdString();

  exec_remote_cmd(cmd);
  refreshInventories();
}

std::string inventory_configuration_widget::getSelectedInventory() const
{
  QListWidgetItem *item = ui->listWidget->currentItem();
  if (!item)
    return "";
  QString current_inventory = item->text();
  return current_inventory.toStdString();
}

} //namespace pal
