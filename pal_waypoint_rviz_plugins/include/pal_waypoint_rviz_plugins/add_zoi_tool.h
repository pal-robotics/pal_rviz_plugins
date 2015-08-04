/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 8/3/2015
 *      Author: victor
 */

#ifndef ADD_ZOI_TOOL_H
#define ADD_ZOI_TOOL_H
#include <pal_waypoint_rviz_plugins/add_point_tool.h>

namespace pal
{

class AddZoiTool: public AddPointTool
{
Q_OBJECT
public:
  AddZoiTool();
  virtual ~AddZoiTool() {}
  virtual void onInitialize();

protected:
  void addPointParam(const std::string &name,
                   double x,
                   double y,
                   double theta);
};

}

#endif // ADD_ZOI_TOOL_H
