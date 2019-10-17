#ifndef ADDPOINTTOOL_H
#define ADDPOINTTOOL_H

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

// Adds tool to add a point to the parameter server
class AddPointTool: public rviz::PoseTool
{
Q_OBJECT
public:
  AddPointTool(const std::string &paramName);
  virtual ~AddPointTool() {}
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);
  virtual void addPointParam(const std::string &name,
                   double x,
                   double y,
                   double theta);

protected:
  ros::NodeHandle _nh;
  tf::TransformListener _tf;
  std::string _paramName;

};


class AddPoiTool: public AddPointTool
{
Q_OBJECT
public:
  AddPoiTool();
  virtual void onInitialize();
};


class AddPodTool: public AddPointTool
{
Q_OBJECT
public:
  AddPodTool();
  virtual void onInitialize();
};
}

#endif // ADDPOINTTOOL_H
