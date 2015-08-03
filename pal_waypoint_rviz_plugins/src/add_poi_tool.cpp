/*
 *  add_poi_tool.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <pal_waypoint_rviz_plugins/add_poi_tool.h>

#include <qinputdialog.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"

namespace pal
{

AddPoiTool::AddPoiTool()
{
}

void AddPoiTool::onInitialize()
{
  AddPointTool::onInitialize();
  setName( "Add Point of Interest");
}

void AddPoiTool::addPointParam(const std::string &name, double x,
                                 double y, double theta)
{
    XmlRpc::XmlRpcValue list;
    list[0] = "submap_0"; // Hard coded value everywhere until multimaps are introduced
    list[1] = name;
    list[2] = x;
    list[3] = y;
    list[4] = theta;
    _nh.setParam("/mmap/poi/submap_0/" + name, list);
}
} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( pal::AddPoiTool, rviz::Tool )
