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

#include "rqt_human_radar/FieldOfViewItem.hpp"

namespace rqt_human_radar
{

FieldOfViewItem::FieldOfViewItem(
  double radius, double angle,
  QGraphicsItem * parent)
: QGraphicsItem(parent), radius_(radius), angle_(angle), color_(Qt::green)
{
  setFlags(QGraphicsItem::ItemStacksBehindParent);
  color_.setAlpha(50);  // Semi-transparent
}

void FieldOfViewItem::setRadius(double radius)
{
  radius_ = radius;
  prepareGeometryChange();
}

void FieldOfViewItem::setAngle(double angle)
{
  angle_ = angle;
  update();
}

void FieldOfViewItem::setColor(const QColor & color)
{
  color_ = color;
  color_.setAlpha(50);  // Ensure semi-transparency
  update();
}

QRectF FieldOfViewItem::boundingRect() const
{
  // Return the bounding rectangle of the sector
  return QRectF(-radius_, -radius_, 2 * radius_, 2 * radius_);
}

QPainterPath FieldOfViewItem::shape() const
{
  QPainterPath path;
  path.moveTo(0, 0);  // Center of the field of view

  // Define the arc as a pie slice (sector)
  path.arcTo(-radius_, -radius_, 2 * radius_, 2 * radius_, -angle_ / 2, angle_);

  // Close the path to form the sector
  path.lineTo(0, 0);
  path.closeSubpath();

  return path;
}

void FieldOfViewItem::paint(
  QPainter * painter, [[maybe_unused]] const QStyleOptionGraphicsItem * option,
  [[maybe_unused]] QWidget * widget)
{
  painter->setBrush(color_);
  painter->setPen(Qt::NoPen);

  // Draw the sector as a pie slice
  QRectF rect(-radius_, -radius_, 2 * radius_, 2 * radius_);
  int startAngle = -angle_ / 2 * 16;  // Convert to 1/16 degree
  int spanAngle = angle_ * 16;        // Convert to 1/16 degree
  painter->drawPie(rect, startAngle, spanAngle);
}

}  // namespace rqt_human_radar
