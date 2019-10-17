#include <pal_navigation_rviz_plugins/map_configuration_widget.h>
#include <pal_navigation_msgs/Acknowledgment.h>
#include <pal_navigation_msgs/RenameMap.h>
#include <pal_navigation_msgs/ListMaps.h>
#include "ui_map_configuration_widget.h"
#include <qmessagebox.h>
#include <qinputdialog.h>
#include <qfiledialog.h>
#include <std_srvs/Empty.h>
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
    //std::string remote_cmd = "ssh pal@" + host + " " + cmd;
    std::string remote_cmd = "sshpass -p pal ssh -o \"StrictHostKeyChecking no\" pal@" + host + " " + cmd;
    return exec(remote_cmd);
  }
}

/**
 * @brief getActiveMap Return name of the map that is currently set as active map
 *                     in the ros master
 */
std::string getActiveMap()
{
  pal_navigation_msgs::ListMaps lm;
  if (ros::service::call("/pal_map_manager/current_map", lm))
  {
    if (lm.response.success && lm.response.maps.size() == 1)
    {
      return lm.response.maps[0];
    }
  }
  return "";
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
  std::string active_map = getActiveMap();
  pal_navigation_msgs::ListMaps lm;

  if (ros::service::call("/pal_map_manager/list_maps", lm) && lm.response.success)
  {
    ui->listWidget->clear();
    for(auto s : lm.response.maps)
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

    bool enable = !lm.response.maps.empty();
    ui->set_active->setEnabled(enable);
    ui->download_map->setEnabled(enable);
    ui->delete_map->setEnabled(enable);
    ui->rename_map->setEnabled(enable);
  }
  else
  {
    QMessageBox::warning(this, "Error Listing Maps", "Unknown error when trying to list maps.");
  }
}

void map_configuration_widget::setActive()
{
    ros::NodeHandle nh;
    ros::ServiceClient clientChangeMap =
            nh.serviceClient<pal_navigation_msgs::Acknowledgment>("/pal_map_manager/change_map");
    pal_navigation_msgs::Acknowledgment srvChangeMap;
    srvChangeMap.request.input = getSelectedMap();
    if ( !clientChangeMap.call(srvChangeMap) )
        QMessageBox::warning(this, "Unable to change active map",
                             srvChangeMap.response.error.c_str());
    else
    {
        ros::ServiceClient clientClearCostmaps =
                nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        std_srvs::Empty srvClearCostmaps;
        clientClearCostmaps.call(srvClearCostmaps);
        refreshMaps();
    }

//    std::string cmd = "source ~/.bashrc; rosservice call /pal_map_manager/change_map \"input: '" +
//                      getSelectedMap() + "'\"";

//    ROS_INFO_STREAM("Running remote cmd: " << cmd);
////  std::string cmd =
////      "ln -sfT " + getMapDir() + "configurations/" + getSelectedMap() +
////      " " + getMapDir() + "/config";
//    exec_remote_cmd(cmd);
//    cmd = "rosservice call /move_base/clear_costmaps \"{}\"";
//    exec_remote_cmd(cmd);
//    refreshMaps();
////  QMessageBox::warning(this, "Reboot needed",
////                       "Robot reboot is needed before new map is used");

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
    cmd = "sshpass -p pal scp -o StrictHostKeyChecking=no -r pal@" + host + ":" + getMapDir() + "configurations/" + map + " " +
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
    cmd = "sshpass -p pal scp -o StrictHostKeyChecking=no -r " + source_dir.toStdString() +
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

  if (ok && name != "")
  {
    ros::NodeHandle nh;
    ros::ServiceClient clientRenameMap =
            nh.serviceClient<pal_navigation_msgs::RenameMap>("/pal_map_manager/rename_map");
    pal_navigation_msgs::RenameMap rename_map_srv;
    rename_map_srv.request.current_map_name = map;
    rename_map_srv.request.new_map_name = name.toStdString();

    if (clientRenameMap.call(rename_map_srv))
    {
      if (rename_map_srv.response.success)
      {
        refreshMaps();
      }
      else
      {
        QMessageBox::warning(this, "Renaming Error", rename_map_srv.response.message.c_str());
      }
    }
    else
    {
      QMessageBox::warning(this, "Connection Error", "Couldn't contact Rename Map Service");
    }
  }
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
