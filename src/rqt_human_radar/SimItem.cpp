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

#include <QFont>

#include "rqt_human_radar/SimItem.hpp"
#include "rqt_human_radar/SimScene.hpp"

namespace rqt_human_radar
{

SimItem::SimItem(rclcpp::Node::SharedPtr node, const std::string & svg_file)
: QGraphicsSvgItem(QString::fromStdString(svg_file)), node_(node)
{
  label_ = new QGraphicsSimpleTextItem();
  label_->setFlags(QGraphicsItem::ItemIgnoresTransformations);
  label_->setText("no label");
  label_->setFont(QFont("Arial", 14));
  label_->setParentItem(this);
  label_->show();
}

QRectF SimItem::boundingRect() const
{
  auto width = physical_width_ * SimScene::pixelsPerMeter;

  double ratio = QGraphicsSvgItem::boundingRect().height() /
    QGraphicsSvgItem::boundingRect().width();

  auto height = width * ratio;

  label_->setPos(-width / 2, height / 2 + 0.05 * SimScene::pixelsPerMeter);

  // redefine the boundingRect to be centered around the origin
  return QRectF(-0.5 * width, -0.5 * height, width, height);
}

void SimItem::paint(
  QPainter * painter,
  [[maybe_unused]] const QStyleOptionGraphicsItem * option,
  [[maybe_unused]] QWidget * widget)
{
  painter->save();
  painter->translate(
    -0.5 * boundingRect().width(),
    -0.5 * boundingRect().height());

  painter->save();
  painter->scale(
    boundingRect().width() / QGraphicsSvgItem::boundingRect().width(),
    boundingRect().height() / QGraphicsSvgItem::boundingRect().height());

  QGraphicsSvgItem::paint(painter, option, widget);
  painter->restore();

  painter->restore();
}

void SimItem::setPhysicalWidth(double size)
{
  physical_width_ = size;
  prepareGeometryChange();
}

void SimItem::setLabel(const std::string & label)
{
  label_->setText(QString::fromStdString(label));
}

}  // namespace rqt_human_radar
