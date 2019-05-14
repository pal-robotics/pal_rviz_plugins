/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef ADD_ZONE_TOOL_H
#define ADD_ZONE_TOOL_H
#include <pal_waypoint_rviz_plugins/add_point_tool.h>

namespace pal
{

class AddZoneTool: public AddPointTool
{
  Q_OBJECT
public:
  AddZoneTool(const std::string &paramName);
  virtual ~AddZoneTool() {}

protected:
  void addPointParam(const std::string &name,
                     double x,
                     double y,
                     double theta);
};

class AddZoiTool : public AddZoneTool
{
  Q_OBJECT
public:
  AddZoiTool();
  virtual void onInitialize();
};

class AddVoTool : public AddZoneTool
{
  Q_OBJECT
public:
  AddVoTool();
  virtual void onInitialize();
};

}



#endif // ADD_ZONE_TOOL_H
