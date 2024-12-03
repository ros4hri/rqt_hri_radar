// Copyright 2024 pal-robotics
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

#include <QSvgRenderer>
#include <QDomDocument>
#include <QGraphicsScene>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rqt_human_radar
{

class EnvironmentLoader
{
public:
  EnvironmentLoader() {}

  void loadMap(rclcpp::Node::SharedPtr node, QGraphicsScene * scene, const std::string & filename);

private:
  QRectF getElementBounds(
    rclcpp::Node::SharedPtr node, const std::string & elementId,
    const std::string & label);

  QSvgRenderer renderer_;
  QDomDocument doc_;
};
}   // namespace rqt_human_radar
