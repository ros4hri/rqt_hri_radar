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
#include <QPainter>
#include <QRectF>
#include <QPointF>
#include <QStyleOptionGraphicsItem>
#include <QWidget>
#include <QPainterPath>
#include <QColor>

#include "SimScene.hpp"

namespace rqt_human_radar
{

class FieldOfViewItem : public QGraphicsItem
{
public:
  explicit FieldOfViewItem(
    double radius = 100.0, double angle = 90.0,
    QGraphicsItem * parent = nullptr);

  void setRadius(double radius);
  void setAngle(double angle);
  void setColor(const QColor & color);

  bool contains([[maybe_unused]] const QPointF &) const override
  {
    return false;  // Always return false to indicate the item doesn't "contain"
                   // any point -- this is necessary for the item *not* to receive
                   // mouse events
  }

protected:
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(
    QPainter * painter, const QStyleOptionGraphicsItem * option,
    QWidget * widget) override;

private:
  double radius_;  // Radius of the field of view
  double angle_;   // Angle of the sector in degrees
  QColor color_;   // Color of the sector
};

}  // namespace rqt_human_radar
