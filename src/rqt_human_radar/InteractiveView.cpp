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

#include "rqt_human_radar/InteractiveView.hpp"

#include <QCoreApplication>
#include <QGraphicsSceneWheelEvent>
#include <QWheelEvent>
#include <rclcpp/rclcpp.hpp>

#include "rqt_human_radar/SimScene.hpp"

namespace rqt_human_radar
{

InteractiveView::InteractiveView(QWidget * parent)
: QGraphicsView(parent)
{
  setDragMode(QGraphicsView::ScrollHandDrag);
  setRenderHint(QPainter::Antialiasing);

  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

void InteractiveView::resetView()
{
  fitInView(
    QRectF(
      -.35 * SimScene::pixelsPerMeter,
      -.8 * SimScene::pixelsPerMeter,
      4.5 * SimScene::pixelsPerMeter,
      1.6 * SimScene::pixelsPerMeter),
    Qt::KeepAspectRatio);         // set the original view
}

void InteractiveView::wheelEvent(QWheelEvent * event)
{
  if (event->modifiers() & Qt::ControlModifier) {
    const double scaleFactor = 1.15;

    if (event->angleDelta().y() > 0) {
      // Zoom in
      if (scaleFactor_ < 3.0) {
        scaleFactor_ *= scaleFactor;
        scale(scaleFactor, scaleFactor);
      }
    } else {
      // Zoom out
      if (scaleFactor_ > 0.5) {
        scaleFactor_ /= scaleFactor;
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
      }
    }
  } else {
    // Scroll up/down
    QGraphicsView::wheelEvent(event);
  }
}

void InteractiveView::resizeEvent(QResizeEvent * event)
{
  QGraphicsView::resizeEvent(event);
  resetView();
}

}  // namespace rqt_human_radar
