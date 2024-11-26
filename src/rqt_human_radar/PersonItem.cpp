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

#include <QGraphicsItem>

#include "rqt_human_radar/FieldOfViewItem.hpp"
#include "rqt_human_radar/LocalObjectItem.hpp"
#include "rqt_human_radar/PersonItem.hpp"
#include "rqt_human_radar/SimScene.hpp"

namespace rqt_human_radar
{

PersonItem::PersonItem(
  rclcpp::Node::SharedPtr node, const std::string & id,
  const std::string & classname,
  const std::string & svg_file)
: SemanticObject(node, id, classname), SimItem(node, svg_file)
{
  setZValue(10);

  setLabel(id_);

  fov_ = new FieldOfViewItem(100.0, 120.0);
  fov_->setRadius(3.5 * SimScene::pixelsPerMeter);
  fov_->setColor(Qt::blue);
  fov_->setParentItem(this);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &PersonItem::objectsInFov);
  timer_->start(100);  // Check overlaps every 100ms
}

void PersonItem::setFovColor(const QColor & color) {fov_->setColor(color);}

void PersonItem::objectsInFov()
{
  QList<QGraphicsItem *> overlappingItems = fov_->collidingItems();

  std::set<std::string> currently_seen_objects_;

  for (QGraphicsItem * item : overlappingItems) {
    if (item != this && dynamic_cast<SemanticObject *>(item)) {
      auto semObj = dynamic_cast<SemanticObject *>(item);
      currently_seen_objects_.insert(semObj->getId());
    }
  }

  std::set<std::string> new_objects;
  std::set<std::string> lost_objects;

  std::set_difference(
    currently_seen_objects_.begin(),
    currently_seen_objects_.end(), last_seen_objects_.begin(),
    last_seen_objects_.end(),
    std::inserter(new_objects, new_objects.begin()));

  std::set_difference(
    last_seen_objects_.begin(), last_seen_objects_.end(),
    currently_seen_objects_.begin(),
    currently_seen_objects_.end(),
    std::inserter(lost_objects, lost_objects.begin()));

  for (const auto & id : new_objects) {
    addProperty("sees", id);
  }

  for (const auto & id : lost_objects) {
    removeProperty("sees", id);
  }

  last_seen_objects_ = currently_seen_objects_;
}

}  // namespace rqt_human_radar
