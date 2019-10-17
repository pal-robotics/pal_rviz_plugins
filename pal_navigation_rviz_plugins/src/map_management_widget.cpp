#include <pal_navigation_rviz_plugins/map_management_widget.h>
#include "ui_map_management_widget.h"
#include <std_srvs/Empty.h>
#include <pal_navigation_msgs/Acknowledgment.h>

namespace pal
{
MapManagementWidget::MapManagementWidget( QWidget* parent )
  :QWidget(parent), ui_(new Ui::MapManagementWidgetUI)
{
  ui_->setupUi(this);
  connect(ui_->save_map_config, SIGNAL(clicked()),this, SLOT(saveMapConfig()));
  connect(ui_->start_mapping, SIGNAL(clicked()),this, SLOT(startMapping()));
  connect(ui_->stop_mapping, SIGNAL(clicked()),this, SLOT(stopMapping()));

  // Not persistent because we're not calling this often
  poi_changed_client_ = nh_.serviceClient<std_srvs::Empty>("/notifyPoisChanged", false);
  navigation_sm_client_ = nh_.serviceClient<pal_navigation_msgs::Acknowledgment>("/pal_navigation_sm", false);
  navigation_sm_state_sub_ = nh_.subscribe("/pal_navigation_sm/state", 1,&MapManagementWidget::smStateCb, this);
}

MapManagementWidget::~MapManagementWidget()
{
  delete ui_;
}

void MapManagementWidget::smStateCb(const pal_navigation_msgs::NavigationStatusConstPtr &msg)
{
  if (msg->status.data == "LOC")
  {
    ui_->start_mapping->setEnabled(true);
    ui_->stop_mapping->setEnabled(false);
  }
  else if (msg->status.data == "MAP")
  {
    ui_->start_mapping->setEnabled(false);
    ui_->stop_mapping->setEnabled(true);
  }
  else
  {
    ui_->start_mapping->setEnabled(false);
    ui_->stop_mapping->setEnabled(false);
  }
}

void MapManagementWidget::saveMapConfig()
{
  std_srvs::Empty srv;
  ui_->status_label->setText("");
  if (!poi_changed_client_.call(srv))
  {
    ui_->status_label->setText("Error!");
    ROS_ERROR_STREAM("Error saving map configuration");
  }
  else
  {
    ui_->status_label->setText("Ok");
    ROS_INFO_STREAM("Map configuration saved on the robot");
  }
}

void MapManagementWidget::startMapping()
{
  pal_navigation_msgs::Acknowledgment srv;
  srv.request.input = "MAP";
  navigation_sm_client_.call(srv);
}

void MapManagementWidget::stopMapping()
{
  pal_navigation_msgs::Acknowledgment srv;
  srv.request.input = "LOC";
  navigation_sm_client_.call(srv);
}

} // end namespace pal

