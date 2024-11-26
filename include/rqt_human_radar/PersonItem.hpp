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

#include <QColor>
#include <QTimer>
#include <set>
#include <string>

#include "SemanticObject.hpp"
#include "SimItem.hpp"

namespace rqt_human_radar
{

class FieldOfViewItem;

class PersonItem : public SemanticObject, public SimItem
{
public:
  enum { Type = UserType + 2 };

  PersonItem(
    rclcpp::Node::SharedPtr node, const std::string & id,
    const std::string & classname = "oro:Agent",
    const std::string & svg_file = "");

  void objectsInFov();

  void setFovColor(const QColor & color);

  int type() const override {return Type;}

private:
  std::set<std::string> last_seen_objects_;

  FieldOfViewItem * fov_;

  QTimer * timer_;
};

}  // namespace rqt_human_radar
