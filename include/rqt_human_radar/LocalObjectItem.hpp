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

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <QGraphicsSceneContextMenuEvent>
#include <QTimer>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "SemanticObject.hpp"
#include "SimItem.hpp"

namespace rqt_human_radar
{

class LocalObjectItem : public SemanticObject, public SimItem
{
public:
  enum { Type = UserType + 5 };

  LocalObjectItem(
    rclcpp::Node::SharedPtr node, const std::string & name,
    const std::string & svg_file,
    const std::string & classname = "oro:Object",
    const std::string & reference_frame_id = "base_link",
    bool randomize_id = false);

  ~LocalObjectItem();

  int type() const override {return Type;}

protected:
  void contextMenuEvent(QGraphicsSceneContextMenuEvent * event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent * event) override;

private:
  void publishTF();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string frame_id_;
  std::string reference_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rqt_human_radar
