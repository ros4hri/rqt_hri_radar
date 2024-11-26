// Copyright 2024 PAL Robotics S.L.
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
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QMenu>
#include <QtMath>

#include "rqt_human_radar/LocalPersonItem.hpp"
#include "rqt_human_radar/SimScene.hpp"

using namespace std::chrono_literals;

namespace rqt_human_radar
{

LocalPersonItem::LocalPersonItem(
  rclcpp::Node::SharedPtr node,
  const std::string & resource_path,
  const std::string & reference_frame_id)
: PersonItem(node, "sim_person_" + generateRandomSuffix(), "oro:Human",
    resource_path + "/res/adult-local.svg"),
  node_(node), frame_id_(id_), reference_frame_id_(reference_frame_id)
{
  setFlags(QGraphicsItem::ItemIsMovable);

  setPhysicalWidth(.3);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Create a timer to periodically broadcast the transform
  timer_ = node_->create_wall_timer(
    50ms, std::bind(&LocalPersonItem::publishTF, this));
}

LocalPersonItem::~LocalPersonItem()
{
  if (timer_) {
    timer_->cancel();
  }
}

void LocalPersonItem::contextMenuEvent(QGraphicsSceneContextMenuEvent * event)
{
  QMenu menu;
  QAction * deleteAction = menu.addAction("Delete");
  QAction * selectedAction = menu.exec(event->screenPos());
  if (selectedAction == deleteAction) {
    scene()->removeItem(this);
    delete this;
  }
}

void LocalPersonItem::publishTF()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = node_->get_clock()->now();
  transformStamped.header.frame_id = reference_frame_id_;
  transformStamped.child_frame_id = frame_id_;
  transformStamped.transform.translation.x =
    pos().x() / SimScene::pixelsPerMeter;
  transformStamped.transform.translation.y =
    -pos().y() / SimScene::pixelsPerMeter;   // Invert Y-axis
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  double rotationDegrees = -rotation();
  double rotationRadians =
    qDegreesToRadians(rotationDegrees);

  // Set the rotation (orientation)
  transformStamped.transform.rotation.x =
    0.0;   // 2D rotation, no x-axis rotation
  transformStamped.transform.rotation.y =
    0.0;   // 2D rotation, no y-axis rotation
  transformStamped.transform.rotation.z =
    std::sin(rotationRadians / 2);   // Quaternion z-component
  transformStamped.transform.rotation.w =
    std::cos(rotationRadians / 2);   // Quaternion w-component

  tf_broadcaster_->sendTransform(transformStamped);
}

void LocalPersonItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
  QGraphicsSvgItem::mouseMoveEvent(event);
  scene()->update();
}

void LocalPersonItem::wheelEvent(QGraphicsSceneWheelEvent * event)
{
  setRotation(rotation() + event->delta() / 10.);
  event->accept();
}

}  // namespace rqt_human_radar
