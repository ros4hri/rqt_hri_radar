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

#include "rqt_human_radar/LocalObjectItem.hpp"
#include "rqt_human_radar/SimScene.hpp"

using namespace std::chrono_literals;

namespace rqt_human_radar
{

LocalObjectItem::LocalObjectItem(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & svg_file,
  const std::string & classname,
  const std::string & reference_frame_id,
  bool randomize_id)
: SemanticObject(node, name, classname, randomize_id),
  SimItem(node, svg_file), node_(node), frame_id_(id_),
  reference_frame_id_(reference_frame_id)
{
  setFlag(QGraphicsItem::ItemIsMovable);
  setLabel(id_);

  setPhysicalWidth(0.2);  // 20cm

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Create a timer to periodically broadcast the transform
  timer_ = node_->create_wall_timer(
    50ms, std::bind(&LocalObjectItem::publishTF, this));
}

LocalObjectItem::~LocalObjectItem()
{
  if (timer_) {
    timer_->cancel();
  }
}

void LocalObjectItem::contextMenuEvent(QGraphicsSceneContextMenuEvent * event)
{
  QMenu menu;
  QAction * deleteAction = menu.addAction("Delete");
  QAction * selectedAction = menu.exec(event->screenPos());
  if (selectedAction == deleteAction) {
    scene()->removeItem(this);
    delete this;
  }
}

void LocalObjectItem::publishTF()
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

  tf_broadcaster_->sendTransform(transformStamped);
}

void LocalObjectItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
  QGraphicsSvgItem::mouseMoveEvent(event);
  scene()->update();
}

}  // namespace rqt_human_radar
