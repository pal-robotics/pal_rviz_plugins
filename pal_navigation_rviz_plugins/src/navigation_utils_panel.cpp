#include <pal_navigation_rviz_plugins/navigation_utils_panel.h>
#include "ui_navigation_utils_panel.h"
#include <laser_pattern_detector_msgs/GetLaserImage.h>
#include <pal_composite_navigation_msgs/GoToFloorPOIAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <QMessageBox>
#include <QInputDialog>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
namespace pal
{
NavigationUtilsPanel::NavigationUtilsPanel(QWidget *parent) :
  rviz::Panel(parent),
  ui(new Ui::NavigationUtilsPanel),
  _ac("/composite_navigation", false)
{
  ui->setupUi(this);
  connect(ui->imgFromScanButton, SIGNAL(clicked()), this, SLOT(imgFromScan()));
  connect(ui->sendNavGoal, SIGNAL(clicked()), this, SLOT(sendNavGoal()));
  connect(ui->cancelNavGoal, SIGNAL(clicked()), this, SLOT(cancelNavGoal()));
  connect(ui->savePosePOI, SIGNAL(clicked()), this, SLOT(savePosePOI()));

}

NavigationUtilsPanel::~NavigationUtilsPanel()
{
  delete ui;
}

void NavigationUtilsPanel::imgFromScan()
{
  laser_pattern_detector_msgs::GetLaserImage srv;
  srv.request.line_between_laser_points = true;
  srv.request.resolution = 100;
  srv.request.max_laser_range = 2;
  if (!ros::service::call("/laser_pattern_detector/get_laser_image", srv))
  {
    QMessageBox::warning(this, "Error connecting", "Unable to get image from laser scan");
    return;
  }

  cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(srv.response.image);
  std::string img_path = "/tmp/image_from_scan.png";
  cv::imwrite(img_path, cv_img->image);
  QMessageBox::information(this, "Image saved", "Image saved to " + QString::fromStdString(img_path));
}

void NavigationUtilsPanel::savePosePOI()
{

  auto pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
                "/robot_pose", ros::Duration(2.0));

  if(pose == NULL){
    ROS_ERROR_STREAM("No robot pose received.");
    return;
  }

  QString nameRegex = QString("([a-z]|[A-Z]|[0-9]|_)+");
  QString name;
  while (true)
  {
    name = QInputDialog::getText(NULL, "Enter name",
                                         "name:",
                                         QLineEdit::Normal);
    if (name.isEmpty())
      return;

    QRegExp validName(nameRegex);
    if (validName.exactMatch(name))
      break;

    QMessageBox box(QMessageBox::Warning, "Invalid name", "Please enter a valid name, containing only "
                "English characters, numbers and the underscore symbol");
    box.exec();
  }

  double yaw = tf::getYaw(pose->pose.pose.orientation);

  XmlRpc::XmlRpcValue list;
  list[0] = "submap_0"; // Hard coded value everywhere until multimaps are introduced
  list[1] = name.toStdString();
  list[2] = pose->pose.pose.position.x;
  list[3] = pose->pose.pose.position.y;
  list[4] = yaw;
  ros::param::set("/mmap/poi/submap_0/" + name.toStdString(), list);
}

void NavigationUtilsPanel::sendNavGoal()
{
  pal_composite_navigation_msgs::GoToFloorPOIGoal goal;
  goal.floor = ui->navFloor->text().toStdString();
  goal.poi = ui->navPOI->text().toStdString();
  _ac.sendGoal(goal,
               boost::bind(&NavigationUtilsPanel::goalDone, this, _1, _2),
               boost::bind(&NavigationUtilsPanel::goalActive, this));

}

void NavigationUtilsPanel::cancelNavGoal()
{
  _ac.cancelAllGoals();
}


void NavigationUtilsPanel::goalActive()
{
  ui->feedbackText->setText(QString::fromStdString(_ac.getState().toString()));
  ui->cancelNavGoal->setEnabled(true);
}

void NavigationUtilsPanel::goalDone(const actionlib::SimpleClientGoalState &state,
                            const pal_composite_navigation_msgs::GoToFloorPOIResultConstPtr &result)
{
  ui->feedbackText->setText(QString::fromStdString(state.toString()));
  ui->cancelNavGoal->setEnabled(false);
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::NavigationUtilsPanel, rviz::Panel )
