// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QDrag>
#include <QMouseEvent>
#include <QMimeData>

#include "rqt_human_radar/KbObjectWidget.hpp"

KbObjectWidget::KbObjectWidget(const QString &file,
        QWidget *parent)
    : QSvgWidget(file, parent)
{
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &RadarCanvas::customContextMenuRequested, this,
    &RadarCanvas::showContextMenu);

}

void KbObjectWidget::showContextMenu(const QPoint &pos) {

    QMenu contextMenu("Manage objects", this);

    auto delete_action = new QAction((QIcon("edit-delete"), "Delete", this);

    connect(delete_action, &QAction::triggered, this, [object, pos, this]() {
            createKbObjectWidget(object.second, pos);
        });
    contextMenu.addAction(delete_action);
    contextMenu.exec(mapToGlobal(pos));
}


void KbObjectWidget::mousePressEvent(QMouseEvent *event) {

    if (event->button() == Qt::LeftButton) {
        QDrag *drag = new QDrag(this);
        QMimeData *mimeData = new QMimeData;
        drag->setMimeData(mimeData);
        drag->setPixmap(grab());
        drag->setHotSpot(event->pos());

        hide();

        drag->exec(Qt::MoveAction);
    }
}
