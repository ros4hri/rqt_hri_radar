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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "SemanticObject.hpp"
#include "SimItem.hpp"

namespace rqt_human_radar
{

class RemoteObjectItem : public SemanticObject, public SimItem
{
public:
  enum { Type = UserType + 6 };

  RemoteObjectItem(
    rclcpp::Node::SharedPtr node, const std::string & name,
    const std::string & svg_file,
    const std::string & classname = "owl:Thing",
    const std::string & reference_frame_id = "base_link",
    bool randomize_id = false, bool is_static = false);

  ~RemoteObjectItem();

  int type() const override {return Type;}

protected:
  std::string frame_id_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

private:
  void updatePosition();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string reference_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rqt_human_radar
