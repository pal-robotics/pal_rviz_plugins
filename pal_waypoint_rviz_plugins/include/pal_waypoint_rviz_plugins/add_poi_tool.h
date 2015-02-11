/*
 *  add_poi_tool.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#ifndef ADD_POI_TOOL_H
#define ADD_POI_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <ros/ros.h>

# include <tf/transform_listener.h>
# include <rviz/default_plugin/tools/pose_tool.h>
#endif

namespace pal
{
class Arrow;
class DisplayContext;
class StringProperty;

class AddPoiTool: public rviz::PoseTool
{
Q_OBJECT
public:
  AddPoiTool();
  virtual ~AddPoiTool() {}
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  void addPoiParam(const std::string &name,
                   double x,
                   double y,
                   double theta);
  ros::NodeHandle _nh;
  tf::TransformListener _tf;

};

}


#endif // ADD_POI_TOOL_H
