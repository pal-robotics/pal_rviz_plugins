/*
 *
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 8/3/2015
 *      Author: victor
 */

#ifndef ADDPODTOOL_H
#define ADDPODTOOL_H

#include <pal_waypoint_rviz_plugins/add_point_tool.h>

namespace pal
{

class AddPodTool: public AddPointTool
{
Q_OBJECT
public:
  AddPodTool();
  virtual ~AddPodTool() {}
  virtual void onInitialize();

protected:
  void addPointParam(const std::string &name,
                   double x,
                   double y,
                   double theta);
};

}


#endif // ADDPODTOOL_H
