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

#include "rqt_human_radar/RadarCanvas.hpp"
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

namespace rqt_human_radar
{

KbObjectWidget::KbObjectWidget(
        const std::string &name,
        const std::string &classname,
        const QString &file,
        rclcpp::Node::SharedPtr node,
        QWidget *parent)
    : QSvgWidget(file, parent), 
        name_(name),
        classname_(classname),
        node_(node),
        tf_broadcaster_(node)
{
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &KbObjectWidget::customContextMenuRequested, this,
    &KbObjectWidget::showContextMenu);

  kb_add_pub_ = node_->create_publisher<std_msgs::msg::String>("/kb/add_fact", 10);
  kb_remove_pub_ = node_->create_publisher<std_msgs::msg::String>("/kb/remove_fact", 10);

  id_ = name_ + "_" + generateRandomSuffix();
  // convert the id to lowercase
  std::transform(id_.begin(), id_.end(), id_.begin(),
    [](unsigned char c){ return std::tolower(c); });

  // create a connection to the ROS2 topic /kb/add and publish the classname
  std_msgs::msg::String msg; 
  msg.data = id_ + " rdf:type " + classname_;

  kb_add_pub_->publish(msg);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &KbObjectWidget::broadcastTransform);
  timer_->start(50);  // milliseconds

}

KbObjectWidget::~KbObjectWidget() {

    toBeDeleted_ = true;

    updateKbVisibility();

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

void KbObjectWidget::reposition() {

    RadarCanvas *canvas = qobject_cast<RadarCanvas *>(parent());

    auto [xOffset, yOffset] = canvas->getOffset();
    int pixelPerMeter = canvas->getPixelPerMeter();

    move(QPoint(xOffset + x_ * pixelPerMeter, yOffset - y_ * pixelPerMeter) - rect().center());

}

void KbObjectWidget::updateKbVisibility() const {

    RadarCanvas *canvas = qobject_cast<RadarCanvas *>(parent());

    if (!canvas->hasFov()) return;

    bool inFov = canvas->isInFov(mapToParent(rect().center()));

    std_msgs::msg::String msg;
    msg.data = "myself oro:sees " + id_;

    if (inFov && !toBeDeleted_) {
        kb_add_pub_->publish(msg);
    } else {
        kb_remove_pub_->publish(msg);
    }
}

void KbObjectWidget::pxPlace(const QPoint &pos) {
    RadarCanvas *canvas = qobject_cast<RadarCanvas *>(parent());

    auto [xOffset, yOffset] = canvas->getOffset();
    int pixelPerMeter = canvas->getPixelPerMeter();

    auto x = (pos.x() - xOffset) / double(pixelPerMeter);
    auto y = (yOffset - pos.y()) / double(pixelPerMeter);


    if ((abs(x_ - x) > 1e-4) || (abs(y_ - y) < 1e-4)) {
        x_ = x;
        y_ = y;

        reposition();
        updateKbVisibility();
    }
}

void KbObjectWidget::resizeEvent([[maybe_unused]] QResizeEvent * event)
{
    reposition();
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

    RadarCanvas *canvas = qobject_cast<RadarCanvas *>(parent());

    auto referenceFrame = canvas->getReferenceFrame();
    if (!referenceFrame) return;

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node_->now();
    transformStamped.header.frame_id = *referenceFrame;
    transformStamped.child_frame_id = id_;

    transformStamped.transform.translation.x = x_;
    transformStamped.transform.translation.y = y_;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_.sendTransform(transformStamped);

}

}  // namespace rqt_human_radar
