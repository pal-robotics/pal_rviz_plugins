/*
 *  waypoint_group_panel.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#include <QDropEvent>
#include <QTimer>
#include <QMenu>
#include <QMessageBox>
#include <QInputDialog>

#include <pal_waypoint_rviz_plugins/waypoint_group_panel.h>
#include <pal_waypoint/DoWaypointNavigationAction.h>

#include "ui_waypoint_group_panel.h"

#include <ros/console.h>

namespace pal
{
QListWidgetItem *createPoiItem(QListWidget *list, const std::string &id, const std::string &name)
{
    QListWidgetItem * item = new QListWidgetItem( QString::fromStdString(name),
                                                  list);
    // Id is transmitted as user data
    item->setData(Qt::UserRole,
                  QVariant(QString::fromStdString(id)));
}


WaypointGroupList::WaypointGroupList(QWidget *parent)
    : QListWidget(parent)
{
    setContextMenuPolicy(Qt::DefaultContextMenu);
}

void WaypointGroupList::contextMenuEvent(QContextMenuEvent *ev)
{
    QListWidgetItem* item = itemAt(ev->pos());
    if (!item)
    {
        QListWidget::contextMenuEvent(ev);
        return;
    }

    QMenu menu;
    boost::shared_ptr<QAction> removeAction(menu.addAction("Remove from group"));
    QAction *action = menu.exec(mapToGlobal(ev->pos()));
    if (action == removeAction.get())
    {
        delete takeItem(row(item));
        Q_EMIT groupPoisChanged();
    }
}

void WaypointGroupList::dropEvent(QDropEvent *ev)
{
    Qt::DropAction action;
    // Move if drop comes from same widget, copy otherwise
    if (ev->source() == this)
        action = Qt::MoveAction;
    else
        action = Qt::CopyAction;

    QDropEvent newev = QDropEvent(ev->pos(), action, ev->mimeData(),
                                  ev->mouseButtons(), ev->keyboardModifiers(), ev->type());
    QListWidget::dropEvent(&newev);
    Q_EMIT groupPoisChanged();
}


WaypointGroupPanel::WaypointGroupPanel( QWidget* parent )
    : rviz::Panel( parent ),
      ui(new Ui::WaypointGroupPanel),
      _poiParam("/mmap/poi/submap_0"),
      _groupParam("/mmap/poigroup/submap_0/"),
      _activeGroup(""),
      _actionClient("/pal_waypoint/navigate", false),
      _goalRunning(false)
{
    ui->setupUi(this);
    connect(ui->groupCombo, SIGNAL(currentIndexChanged(QString)),this, SLOT(groupChanged(QString)));
    connect(ui->newGroupBut, SIGNAL(clicked()),this, SLOT(newGroup()));
    connect(ui->delGroupBut, SIGNAL(clicked()),this, SLOT(delGroup()));
    connect(ui->runGroupBut, SIGNAL(clicked()),this, SLOT(runGroup()));
    connect(ui->stopGroupBut, SIGNAL(clicked()),this, SLOT(stopGroup()));
    connect(ui->groupList, SIGNAL(groupPoisChanged()),this, SLOT(saveActiveGroup()));

    updateButtonStatus();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updatePoiList()));
    timer->start(5000);
    updatePoiList();

    XmlRpc::XmlRpcValue pois;
    if (_nh.getParamCached(_groupParam, pois))
    {
        for(XmlRpc::XmlRpcValue::iterator it = pois.begin(); it != pois.end(); ++it)
        {
            ui->groupCombo->addItem(QString::fromStdString(it->first));
        }
    }
}

WaypointGroupPanel::~WaypointGroupPanel()
{
    delete ui;
}

void WaypointGroupPanel::updatePoiList()
{
    XmlRpc::XmlRpcValue pois;
    if (!_nh.getParamCached(_poiParam, pois))
        return;
    ui->poiList->clear();
    for(XmlRpc::XmlRpcValue::iterator it = pois.begin(); it != pois.end(); ++it)
    {
        std::string id = it->first;
        XmlRpc::XmlRpcValue poi = it->second;
        std::string name = poi[1];
        QListWidgetItem *item = createPoiItem(ui->poiList, id, name);
        ui->poiList->addItem(item);
    }
}

void WaypointGroupPanel::groupChanged(const QString &group)
{
    saveActiveGroup();

    _activeGroup = group;
    updateButtonStatus();

    XmlRpc::XmlRpcValue groupParam;
    ui->groupList->clear();
    if (!_nh.getParamCached(_groupParam + "/" + group.toStdString(), groupParam)
            || groupParam.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return;

    for(int i = 0; i < groupParam.size(); ++i)
    {
        std::string poiId = groupParam[i];
        XmlRpc::XmlRpcValue param;
        if (_nh.getParamCached(_poiParam + "/" + poiId, param))
            ui->groupList->addItem(createPoiItem(ui->groupList, poiId, param[1]));
    }
}

void WaypointGroupPanel::newGroup()
{
    QString group = QInputDialog::getText(this, "Enter group name", "Group",QLineEdit::Normal);
    ui->groupCombo->addItem(group);
    ui->groupCombo->setCurrentIndex(ui->groupCombo->count() - 1);
}

void WaypointGroupPanel::delGroup()
{
    std::string group = _activeGroup.toStdString();
    _nh.deleteParam(_groupParam + "/" + group);
    _activeGroup = ""; // So removeItem doesn't trigger groupChanged and saves the params
    ui->groupCombo->removeItem(ui->groupCombo->currentIndex());
}

void WaypointGroupPanel::runGroup()
{
    if (!_actionClient.waitForServer(ros::Duration(1)))
    {
        QMessageBox::warning(this, "Can't connect to server",
                             "Errors connecting to waypoint group server.");
        return;
    }

    pal_waypoint::DoWaypointNavigationGoal goal;
    goal.group = _activeGroup.toStdString();
    _actionClient.sendGoal(goal,
                           boost::bind(&WaypointGroupPanel::goalDone, this, _1, _2),
                           boost::bind(&WaypointGroupPanel::goalActive, this));


}

void WaypointGroupPanel::stopGroup()
{
    _actionClient.cancelGoal();
}

void WaypointGroupPanel::saveActiveGroup()
{
    if (_activeGroup.isEmpty())
        return;

    std::vector<std::string> pois;
    for(int i=0; i < ui->groupList->count(); ++i)
    {
        QListWidgetItem *item = ui->groupList->item(i);
        std::string id = item->data(Qt::UserRole).toString().toStdString();
        pois.push_back(id);
    }
    _nh.setParam(_groupParam + "/" + _activeGroup.toStdString(),
                 pois);

}

void WaypointGroupPanel::updateButtonStatus() const
{
    bool hasGroup = !_activeGroup.isEmpty();
    ui->groupList->setEnabled(hasGroup);
    ui->delGroupBut->setEnabled(hasGroup);

    ui->runGroupBut->setEnabled(hasGroup && !_goalRunning);
    ui->stopGroupBut->setEnabled(hasGroup && _goalRunning);
}

void WaypointGroupPanel::goalActive()
{
    _goalRunning = true;
    updateButtonStatus();
}

void WaypointGroupPanel::goalDone(const actionlib::SimpleClientGoalState& state,
                                  const pal_waypoint::DoWaypointNavigationResultConstPtr& result)
{
    _goalRunning = false;
    updateButtonStatus();
}
} // end namespace pal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::WaypointGroupPanel, rviz::Panel )
