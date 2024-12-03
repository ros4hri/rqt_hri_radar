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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rqt_human_radar/LocalObjectItem.hpp"
#include "rqt_human_radar/SimScene.hpp"
#include "rqt_human_radar/SPOEditor.hpp"

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
: SimItem(node, svg_file),
  SemanticObject(node, name, classname, randomize_id),
  node_(node),
  frame_id_(id_),
  reference_frame_id_(reference_frame_id),
  animation_(new QPropertyAnimation(this, "pos"))
{
  init();
}

LocalObjectItem::LocalObjectItem(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & classname,
  const std::string & reference_frame_id,
  bool randomize_id)
: SimItem(node),
  SemanticObject(node, name, classname, randomize_id),
  node_(node),
  frame_id_(id_),
  reference_frame_id_(reference_frame_id),
  animation_(new QPropertyAnimation(this, "pos"))
{
  init();
}

void LocalObjectItem::init()
{
  setFlag(QGraphicsItem::ItemIsMovable);
  setLabel(id_);

  setPhysicalWidth(0.2);   // 20cm

  animation_->setDuration(1000);
  animation_->setEasingCurve(QEasingCurve::InOutQuad);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a timer to periodically broadcast the transform
  timer_ = node_->create_wall_timer(
    50ms, std::bind(&LocalObjectItem::publishTF, this));

  pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/interaction_sim/" + id_ + "/move_to",
    10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      moveTo(msg);
    });
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
  QAction * rdfEditAction = menu.addAction("Edit RDF properties");
  menu.addSeparator();
  QAction * deleteAction = menu.addAction("Delete");

  QAction * selectedAction = menu.exec(event->screenPos());

  if (selectedAction == rdfEditAction) {
    auto rdfDialog = new SPOEditor(id_);

    rdfDialog->setTriples(getStaticTriples());

    if (rdfDialog->exec() == QDialog::Accepted) {
      updateStaticTriples(rdfDialog->getTriples());
    }

  } else if (selectedAction == deleteAction) {
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

void LocalObjectItem::moveTo(const geometry_msgs::msg::PoseStamped::SharedPtr new_pos)
{
  try {
    auto transformStamped = tf_buffer_->lookupTransform(
      reference_frame_id_, new_pos->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::doTransform(*new_pos, transformed_pose, transformStamped);

    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "Moving to " << transformed_pose.pose.position.x <<
        "m, " << transformed_pose.pose.position.y << "m in frame " <<
        reference_frame_id_);

    QPointF newPosition(transformed_pose.pose.position.x * SimScene::pixelsPerMeter,
      -transformed_pose.pose.position.y * SimScene::pixelsPerMeter);

    // we can not update the position directly from the ROS callback thread,
    // so we use invokeMethod to do it in the main thread
    QMetaObject::invokeMethod(
      this, [this, newPosition]() {
        animation_->stop();
        animation_->setStartValue(pos());   // Current position
        animation_->setEndValue(newPosition);   // Target position
        animation_->start();
      });
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform: %s", ex.what());
  }
}


}  // namespace rqt_human_radar
