/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 8/3/2015
 *      Author: victor
 */


#include <pal_waypoint_rviz_plugins/add_point_tool.h>

#include <qinputdialog.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
namespace pal
{

AddPointTool::AddPointTool()
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
  QString name = QInputDialog::getText(NULL, "Enter name",
                                       "name:",
                                       QLineEdit::Normal);
  if (name.isEmpty())
    return;
  addPointParam(name.toStdString(), mapPose.pose.position.x,
              mapPose.pose.position.y, yaw);
}

} // end namespace rviz
