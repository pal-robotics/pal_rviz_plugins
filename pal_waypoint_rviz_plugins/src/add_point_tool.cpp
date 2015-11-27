/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 8/3/2015
 *      Author: victor
 */


#include <pal_waypoint_rviz_plugins/add_point_tool.h>

#include <QInputDialog>
#include <QMessageBox>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
namespace pal
{

AddPointTool::AddPointTool(const std::string &paramName)
  : _paramName(paramName)
{
}

void AddPointTool::onInitialize()
{
  rviz::PoseTool::onInitialize();
}

void AddPointTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::PoseStamped pose, mapPose;
  pose.header.frame_id = fixed_frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(quat,
                        pose.pose.orientation);
  _tf.transformPose("/map", pose, mapPose);

  tf::quaternionMsgToTF(pose.pose.orientation, quat);
  double yaw = tf::getYaw(pose.pose.orientation);

  QString nameRegex = QString("([a-z]|[A-Z]|[0-9]|_)+");
  QString name;
  while (true)
  {
    name = QInputDialog::getText(NULL, "Enter name",
                                         "name:",
                                         QLineEdit::Normal);
    if (name.isEmpty())
      return;

    QRegExp validName(nameRegex);
    if (validName.exactMatch(name))
      break;

    QMessageBox box(QMessageBox::Warning, "Invalid name", "Please enter a valid name, containing only "
                "English characters, numbers and the underscore symbol");
    box.exec();
  }

  addPointParam(name.toStdString(), mapPose.pose.position.x,
              mapPose.pose.position.y, yaw);
}


void AddPointTool::addPointParam(const std::string &name, double x,
                                 double y, double theta)
{
    XmlRpc::XmlRpcValue list;
    list[0] = "submap_0"; // Hard coded value everywhere until multimaps are introduced
    list[1] = name;
    list[2] = x;
    list[3] = y;
    list[4] = theta;
    _nh.setParam(_paramName + name, list);
}

AddPoiTool::AddPoiTool() :
  AddPointTool("/mmap/poi/submap_0/")
{
}

void AddPoiTool::onInitialize()
{
  AddPointTool::onInitialize();
  setName( "Point of Interest");
}


AddPodTool::AddPodTool()
  : AddPointTool("/mmap/pod/submap_0/" )
{
}

void AddPodTool::onInitialize()
{
  AddPointTool::onInitialize();
  setName( "Point of Direction");
}



} // end namespace pal


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( pal::AddPoiTool, rviz::Tool )

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( pal::AddPodTool, rviz::Tool )

