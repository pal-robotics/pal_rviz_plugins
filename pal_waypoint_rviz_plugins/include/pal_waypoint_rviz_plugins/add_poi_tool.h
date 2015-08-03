/*
 *  add_poi_tool.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#ifndef ADD_POI_TOOL_H
#define ADD_POI_TOOL_H
#include <pal_waypoint_rviz_plugins/add_point_tool.h>

namespace pal
{

class AddPoiTool: public AddPointTool
{
Q_OBJECT
public:
  AddPoiTool();
  virtual ~AddPoiTool() {}
  virtual void onInitialize();

protected:
  void addPointParam(const std::string &name,
                   double x,
                   double y,
                   double theta);
};

}


#endif // ADD_POI_TOOL_H
