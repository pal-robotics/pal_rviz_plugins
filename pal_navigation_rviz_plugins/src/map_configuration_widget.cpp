/*
 *  map_configuration_widget.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 11/9/2015
 *      Author: victor
 */

#include <pal_navigation_rviz_plugins/map_configuration_widget.h>
#include "ui_map_configuration_widget.h"
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

std::string getMapDir()
{
  std::string host = ros::master::getHost();
  if (host == "localhost")
    return "~/.pal/maps/";
  else
    return "\\$HOME/.pal/maps/";
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

/**
 * @brief getActiveMap Return name of the map that is currently set as active map
 *                     in the ros master
 */
std::string getActiveMap()
{
  std::string get_active_map_cmd = "readlink " + getMapDir() + "config | xargs basename";
  std::string active_map = exec_remote_cmd(get_active_map_cmd);
  boost::replace_all(active_map, "\n", "");
  return active_map;
}

map_configuration_widget::map_configuration_widget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::map_configuration_widgetUI)
{
  ui->setupUi(this);
  connect(ui->refresh_maps, SIGNAL(clicked()), this, SLOT(refreshMaps()));
  connect(ui->set_active, SIGNAL(clicked()), this, SLOT(setActive()));
  connect(ui->download_map, SIGNAL(clicked()), this, SLOT(downloadMap()));
  connect(ui->upload_map, SIGNAL(clicked()), this, SLOT(uploadMap()));
  connect(ui->delete_map, SIGNAL(clicked()), this, SLOT(deleteMap()));
  connect(ui->rename_map, SIGNAL(clicked()), this, SLOT(renameMap()));

  std::string host = ros::master::getHost();
  ui->label->setText(QString::fromStdString(host + " maps:"));
}

map_configuration_widget::~map_configuration_widget()
{
  delete ui;
}

void map_configuration_widget::refreshMaps()
{
  ui->listWidget->clear();
  std::string cmd =
      "ls " + getMapDir() + "configurations/*/map.yaml | xargs -l1 dirname | xargs -l1 basename";

  std::string output = exec_remote_cmd(cmd);

  std::string active_map = getActiveMap();
  std::vector<std::string> maps;
  boost::split(maps, output, boost::is_any_of("\n"));

  for(auto s : maps)
  {
    if (s == "")
      continue;
    QString map = QString::fromStdString(s);
    QListWidgetItem *item = new QListWidgetItem(map, ui->listWidget);
    ui->listWidget->addItem(item);
    if (s == active_map)
    {
      QFont f = item->font();
      f.setBold(true);
      item->setFont(f);
      ui->listWidget->setCurrentItem(item);
    }
  }

  if (!ui->listWidget->currentItem())
    ui->listWidget->setCurrentRow(0);

  bool enable = !maps.empty();
  ui->set_active->setEnabled(enable);
  ui->download_map->setEnabled(enable);
  ui->delete_map->setEnabled(enable);
  ui->rename_map->setEnabled(enable);
}

void map_configuration_widget::setActive()
{
  std::string cmd =
      "ln -sfT " + getMapDir() + "configurations/" + getSelectedMap() +
      " " + getMapDir() + "/config";
  exec_remote_cmd(cmd);
  refreshMaps();
  QMessageBox::warning(this, "Reboot needed",
                       "Robot reboot is needed before new map is used");
}

void map_configuration_widget::downloadMap()
{
  std::string map = getSelectedMap();
  std::string host = ros::master::getHost();

  QString target_dir = QFileDialog::getExistingDirectory(this, "Select directory to save map to",
                                                         "/home", QFileDialog::ShowDirsOnly);

  if (target_dir.isNull())
    return;

  std::string cmd;
  if (host == "localhost")
    cmd = "cp -r " + getMapDir() + "configurations/" + map + " " +
          target_dir.toStdString();
  else
    cmd = "scp -r pal@" + host + ":" + getMapDir() + "configurations/" + map + " " +
          target_dir.toStdString();
  exec(cmd);
}

void map_configuration_widget::uploadMap()
{
  std::string host = ros::master::getHost();
  QString source_dir;
  do
  {
    if (!source_dir.isNull())
      QMessageBox::warning(this, "Invalid map",
                           "Selected directory doesn't contain a valid map. \
                           Directory should contain a valid map.yaml file and map image.");

                           source_dir = QFileDialog::getExistingDirectory(this, "Select map to upload",
                                                                          "/home");
  }while (!source_dir.isNull() && !boost::filesystem::exists(source_dir.toStdString() + "/map.yaml"));

  if (source_dir.isNull())
    return;

  std::string cmd;
  if (host == "localhost")
    cmd = "cp -r " + source_dir.toStdString() + " " +
          getMapDir() + "configurations/";
  else
    cmd = "scp -r " + source_dir.toStdString() +
          " pal@" + host + ":" + getMapDir() + "configurations/";
  exec(cmd);
  refreshMaps();
}

void map_configuration_widget::deleteMap()
{
  std::string map = getSelectedMap();
  if (map == "")
    return;

  if (map == getActiveMap())
  {
    QMessageBox::warning(this, "Can't delete active map",
                         "Cannot delete active map, change active map before attempting to delete "
                         + QString::fromStdString(map));
    return;
  }

  if (QMessageBox::question(this, "Delete map",
                            "Delete map \"" + QString::fromStdString(map) + "\"?",
                            QMessageBox::StandardButton::Ok | QMessageBox::StandardButton::Cancel)
      != QMessageBox::StandardButton::Ok)
    return;

  std::string cmd = "rm -rf " + getMapDir() + "configurations/" + map;
  exec_remote_cmd(cmd);
  refreshMaps();
}

void map_configuration_widget::renameMap()
{
  std::string map = getSelectedMap();


  if (map == getActiveMap())
  {
    QMessageBox::warning(this, "Can't rename active map",
                         "Cannot rename active map, change active map before attempting to rename "
                         + QString::fromStdString(map));
    return;
  }

  QRegExp regexp("[A-Z0-9_]+",Qt::CaseInsensitive);
  QString name = QString::fromStdString(map);
  bool ok = false;
  do
  {
    name = QInputDialog::getText(this, "Enter new map name",
                                 QString::fromStdString("Please enter new name for map " + map +
                                                        " (only letters, numbers and underscores)"),
                                 QLineEdit::EchoMode::Normal, name, &ok);
  }
  while (ok && !regexp.exactMatch(name));

  std::string cmd = "mv " + getMapDir() + "configurations/" + map + " " +
                    getMapDir() + "configurations/" + name.toStdString();

  exec_remote_cmd(cmd);
  refreshMaps();
}

std::string map_configuration_widget::getSelectedMap() const
{
  QListWidgetItem *item = ui->listWidget->currentItem();
  if (!item)
    return "";
  QString current_map = item->text();
  return current_map.toStdString();
}

} //namespace pal
