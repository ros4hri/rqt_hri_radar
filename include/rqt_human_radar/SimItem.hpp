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

#include <QGraphicsItem>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsSvgItem>
#include <QPainter>
#include <QRectF>
#include <QStyleOptionGraphicsItem>
#include <QWidget>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rqt_human_radar
{

class SimItem : public QGraphicsSvgItem
{
public:
  enum { Type = UserType + 1 };

  SimItem(rclcpp::Node::SharedPtr node, const std::string & svg_file);
  SimItem(rclcpp::Node::SharedPtr node);

  /**
   * @brief Set the size of the object in meters.
   *
   * @param size target width in meters
   */
  void setPhysicalWidth(double size);

  QRectF boundingRect() const override;

  void paint(QPainter *, const QStyleOptionGraphicsItem *, QWidget *) override;

  int type() const override {return Type;}

  void setLabel(const std::string & label);

  void showLabel(bool show) {show ? label_->show() : label_->hide();}

protected:
  void init();
  rclcpp::Node::SharedPtr node_;
  double physical_width_ = 1.0;  // m

  QGraphicsSimpleTextItem * label_;
};

}  // namespace rqt_human_radar
