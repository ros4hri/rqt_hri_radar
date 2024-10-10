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

#include <QAction>
#include <QDrag>
#include <QMenu>
#include <QMimeData>
#include <QMouseEvent>


#include <random>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rqt_human_radar/KbObjectWidget.hpp"

std::string generateRandomSuffix(size_t length = 5) {
    static const char charset[] = "abcdefghijklmnopqrstuvwxyz";
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<size_t> dist(0, sizeof(charset) - 2);

    std::string result;
    result.reserve(length);
    for (size_t i = 0; i < length; ++i) {
        result += charset[dist(rng)];
    }
    return result;
}


KbObjectWidget::KbObjectWidget(
        const std::string &classname,
        const QString &file,
        rclcpp::Node::SharedPtr node,
        int pixelPerMeter,
        std::optional<std::string> referenceFrame,
        QWidget *parent)
    : QSvgWidget(file, parent), 
        classname_(classname),
        node_(node),
        tf_broadcaster_(node),
        pixelPerMeter_(pixelPerMeter),
        referenceFrame_(referenceFrame)
{
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &KbObjectWidget::customContextMenuRequested, this,
    &KbObjectWidget::showContextMenu);

  kb_add_pub_ = node_->create_publisher<std_msgs::msg::String>("/kb/add_fact", 10);
  kb_remove_pub_ = node_->create_publisher<std_msgs::msg::String>("/kb/remove_fact", 10);

  id_ = classname_ + "_" + generateRandomSuffix();
  // convert the id to lowercase
  std::transform(id_.begin(), id_.end(), id_.begin(),
    [](unsigned char c){ return std::tolower(c); });

  // create a connection to the ROS2 topic /kb/add and publish the classname
  std_msgs::msg::String msg; 
  msg.data = id_ + " rdf:type " + classname_;

  kb_add_pub_->publish(msg);
  
  // TODO: get this value directly from the RadarCanvas and make sure it is updated
  // when the window is resized
  // xOffset is meant to be fixed, while yOffset depends
  // on the window size
  xOffset_ = 50;
  yOffset_ = parent->size().height() / 2;

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &KbObjectWidget::broadcastTransform);
  timer_->start(50);  // milliseconds

}

KbObjectWidget::~KbObjectWidget() {
    std_msgs::msg::String msg;
    msg.data = id_ + " rdf:type " + classname_;

    kb_remove_pub_->publish(msg);
}

void KbObjectWidget::showContextMenu(const QPoint &pos) {

    QMenu contextMenu("Manage objects", this);

    auto delete_action = new QAction(QIcon::fromTheme("edit-delete"), "Delete", this);

    connect(delete_action, &QAction::triggered, this, [this]() {
            this->deleteLater();
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


void KbObjectWidget::broadcastTransform() {

    if (!referenceFrame_) return;

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node_->now();
    transformStamped.header.frame_id = *referenceFrame_;
    transformStamped.child_frame_id = id_;

    transformStamped.transform.translation.x = (x() - xOffset_)/float(pixelPerMeter_);
    transformStamped.transform.translation.y = -(y() - yOffset_)/float(pixelPerMeter_);
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_.sendTransform(transformStamped);

}
