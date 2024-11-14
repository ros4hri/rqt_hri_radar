// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

#include <QAction>
#include <QMenu>
#include <QMouseEvent>
#include <QtMath>

#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For transforming geometry_msgs types; if not included, templates for doTransform are not found

#include "rqt_human_radar/RadarCanvas.hpp"
#include "rqt_human_radar/PersonWidget.hpp"

using namespace std::chrono_literals;

const double HUMAN_SIZE = .30;   // physical size (in m) of the human along its x-axis

namespace rqt_human_radar
{

PersonWidget::PersonWidget(
  const std::string & id,
  const std::string & frame,
  const QString & svgFile,
  const QString & svgFileSelected,
  std::shared_ptr<tf2_ros::Buffer> tfBuffer,
  rclcpp::Node::SharedPtr node,
  QWidget * parent)
: QSvgWidget(svgFile, parent),
  id_(id),
  frame_(frame),
  tfBuffer_(tfBuffer),
  node_(node),
  renderer_(new QSvgRenderer(svgFile, this)),
  renderer_selected_(new QSvgRenderer(svgFileSelected, this))
{
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(
    this, &PersonWidget::customContextMenuRequested, this,
    &PersonWidget::showContextMenu);

  svg_aspect_ratio_ = double(renderer_->defaultSize().width()) / renderer_->defaultSize().height();



  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &PersonWidget::reposition);
  timer_->start(50);  // milliseconds

}

PersonWidget::~PersonWidget()
{
  //updateKbVisibility();
}

void PersonWidget::showContextMenu(const QPoint & pos)
{
  QMenu contextMenu("Manage person", this);

  auto delete_action = new QAction(QIcon::fromTheme("edit-delete"), "Delete", this);

  connect(
    delete_action, &QAction::triggered, this, [this]() {
      this->deleteLater();
    });
  contextMenu.addAction(delete_action);
  contextMenu.exec(mapToGlobal(pos));
}


void PersonWidget::reposition() {

  geometry_msgs::msg::Vector3Stamped rotatedVector;
  geometry_msgs::msg::Vector3Stamped vector;
  geometry_msgs::msg::TransformStamped personTrans;

  vector.vector.x = 1.0;
  vector.vector.y = 0.0;
  vector.vector.z = 0.0;
  vector.header.stamp = rclcpp::Time();
  vector.header.frame_id = frame_;
  
  RadarCanvas * canvas = qobject_cast<RadarCanvas *>(parent());
  int pixelPerMeter = canvas->getPixelPerMeter();

  auto referenceFrame = canvas->getReferenceFrame();
  if (!referenceFrame) {return;}

  auto [xOffset, yOffset] = canvas->getOffset();

  QPoint personCenter;

  try {
      personTrans = tfBuffer_->lookupTransform(
          *referenceFrame, frame_, rclcpp::Time(0),
          rclcpp::Duration(std::chrono::nanoseconds(10ms)));
  
      x_ = personTrans.transform.translation.x;
      y_ = personTrans.transform.translation.y;

      tf2::doTransform(vector, rotatedVector, personTrans);

      theta_ = M_PI - std::atan2(-(rotatedVector.vector.y), -(rotatedVector.vector.x));

      personCenter = QPoint(x_ * pixelPerMeter + xOffset,
                            - y_ * pixelPerMeter + yOffset);

  } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_->get_logger(), "%s", ex.what());
  }

  move(personCenter - head_offset_);
}

// void PersonWidget::updateKbVisibility() const
// {
//   RadarCanvas * canvas = qobject_cast<RadarCanvas *>(parent());
// 
//   if (!canvas->hasFov()) {return;}
// 
//   bool inFov = canvas->isInFov(mapToParent(rect().center()));
// 
//   std_msgs::msg::String msg;
//   msg.data = "myself oro:sees " + id_;
// 
//   if (inFov && !toBeDeleted_) {
//     kb_add_pub_->publish(msg);
//   } else {
//     kb_remove_pub_->publish(msg);
//   }
// }

void PersonWidget::updateGeometry(QPainter& painter)
{


  // our widget geometry is a square of HUMAN_SIZE meters * sqrt(2) (to account
  // for rotations) + the size of the label
  int iconSize = HUMAN_SIZE * previous_pixelPerMeter_;
  QRect iconRect = QRect(0, 0, 
                         iconSize * qSqrt(2),
                         iconSize * qSqrt(2));

  QString p_id = QString::fromStdString(id_);

  QRect labelRect = painter.boundingRect(QRect(0,0,rect().width()*4, rect().height()),
                                         Qt::AlignLeft,
                                         p_id);

  auto fullBB = iconRect.united(labelRect);

  int svg_w, svg_h;
  if (svg_aspect_ratio_ > 1) {
      svg_w = iconSize;
      svg_h = iconSize/svg_aspect_ratio_;
  } else {
      svg_w = iconSize * svg_aspect_ratio_;
      svg_h = iconSize;
  }
  // the bounding rect of the SVG should be aligned with the top of
  // the full bounding box (ie, y=0), and horizontally centered
  svg_bounds_ = QRect((fullBB.width() - svg_w) / 2, 0, svg_w, svg_h);

  head_offset_ = QPoint(fullBB.center().x(), iconRect.center().y());

  setGeometry(fullBB);
}

void PersonWidget::paintEvent([[maybe_unused]] QPaintEvent *event) {

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setFont(QFont("Arial", 10));

  RadarCanvas * canvas = qobject_cast<RadarCanvas *>(parent());

  int pixelPerMeter = canvas->getPixelPerMeter();
  if (pixelPerMeter != previous_pixelPerMeter_) {
    previous_pixelPerMeter_ = pixelPerMeter;
    updateGeometry(painter);
  }

  painter.save();

  painter.translate(svg_bounds_.center());
  painter.rotate(qRadiansToDegrees(theta_));
  painter.translate(-svg_bounds_.center());

  // Render the SVG manually with the transformed painter
  if (selected_) {
      renderer_selected_->render(&painter, svg_bounds_);
  }
  else {
      renderer_->render(&painter, svg_bounds_);
  }

  painter.restore();

  if (show_id_) {
    painter.save();
    painter.setPen(Qt::black);
    QString identificator = QString::fromStdString(id_);
    painter.drawText(QPoint(rect().x(), svg_bounds_.bottom() + 10), identificator);
    painter.restore();
  }
}

void PersonWidget::resizeEvent([[maybe_unused]] QResizeEvent * event)
{
  reposition();
}

void PersonWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
      selected_ = !selected_;
  }
}



}  // namespace rqt_human_radar
