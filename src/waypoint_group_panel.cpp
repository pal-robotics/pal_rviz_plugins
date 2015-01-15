/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QDropEvent>
#include <QTimer>
#include <QMenu>
#include <geometry_msgs/Twist.h>

#include <pal_waypoint_rviz_plugins/waypoint_group_panel.h>
#include "ui_waypoint_group_panel.h"

#include <ros/console.h>
namespace pal
{

WaypointGroupList::WaypointGroupList(QWidget *parent)
    : QListWidget(parent)
{
    setContextMenuPolicy(Qt::DefaultContextMenu);
}

WaypointGroupPanel::~WaypointGroupPanel()
{
    delete ui;
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
        delete takeItem(row(item));

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
}

WaypointGroupPanel::WaypointGroupPanel( QWidget* parent )
    : rviz::Panel( parent ),
      ui(new Ui::WaypointGroupPanel),
      _poiParam("/mmap/poi/submap_0")
{
    ui->setupUi(this);
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updatePoiList()));
    timer->start(5000);
    updatePoiList();
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
        QListWidgetItem *item = new QListWidgetItem( QString::fromStdString(name),
                                                     ui->poiList);
        // Id is transmitted as user data
        item->setData(Qt::UserRole,
                      QVariant(QString::fromStdString(id)));
        ui->poiList->addItem(item);
    }

    std::cout << "count is " << ui->groupList->count() << std::endl;
    for(int i=0; i < ui->groupList->count(); ++i)
    {
        QListWidgetItem *item = ui->groupList->item(i);
        std::cout << item->data(Qt::UserRole).toString().toStdString() << std::endl;
    }
}
} // end namespace pal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::WaypointGroupPanel, rviz::Panel )

