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

#include <QRectF>
#include <QStyleOptionGraphicsItem>
#include <QWidget>
#include <QPainter>
#include <string>
#include <memory>

#include <hri/hri.hpp>
#include <rclcpp/rclcpp.hpp>

#include "PersonItem.hpp"

namespace rqt_human_radar
{

class RemotePersonItem : public PersonItem
{
public:
  enum { Type = UserType + 4 };

  RemotePersonItem(
    rclcpp::Node::SharedPtr node, hri::ConstPersonPtr person,
    const std::string & resource_path,
    const std::string & reference_frame_id = "base_link",
    bool is_static = false);

  ~RemotePersonItem();

  int type() const override {return Type;}

  bool isVisuallyTracked() const {return person_->face() || person_->body();}

protected:
  std::string frame_id_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

private:
  void updatePosition();

  rclcpp::Node::SharedPtr node_;
  hri::ConstPersonPtr person_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string reference_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rqt_human_radar
