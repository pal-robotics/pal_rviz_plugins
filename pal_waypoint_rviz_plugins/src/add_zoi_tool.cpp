/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 8/3/2015
 *      Author: victor
 */

#include <pal_waypoint_rviz_plugins/add_zoi_tool.h>

#include <qinputdialog.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include <strings.h>


namespace pal
{

AddZoiTool::AddZoiTool()
{
}

void AddZoiTool::onInitialize()
{
  AddPointTool::onInitialize();
  setName( "Add Zone of Interest");
}

void AddZoiTool::addPointParam(const std::string &name, double x,
                               double y, double theta)
{

  int vertexs = QInputDialog::getInt(NULL, "How many vertexs for the zone?",
                                     "vertexs:", 3, 3, 10);

  XmlRpc::XmlRpcValue list;
  list[0] = "submap_0"; // Hard coded value everywhere until multimaps are introduced
  list[1] = name;
  list[4] = theta;
  double radius = 2.0;
  double angle = 2*M_PI/vertexs;
  for (int i = 0; i < vertexs; i++)
  {
    std::stringstream ss;
    ss << "/mmap/zoi/submap_0/" << name << "_" << i;
    list[2] = x + radius * sin(i * angle);
    list[3] = y + radius * cos(i * angle);

    _nh.setParam(ss.str(), list);
  }
}
} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( pal::AddZoiTool, rviz::Tool )


