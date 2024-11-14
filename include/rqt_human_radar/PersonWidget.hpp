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

#ifndef RQT_HUMAN_RADAR__PERSONWIDGET_HPP_
#define RQT_HUMAN_RADAR__PERSONWIDGET_HPP_

#include <QTimer>
#include <QSvgWidget>
#include <QPainter>
#include <QSvgRenderer>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace rqt_human_radar
{

class PersonWidget : public QSvgWidget
{
  Q_OBJECT

public:
  PersonWidget(
    const std::string & id,
    const std::string & frame,
    const QString & svgFile,
    const QString & svgFileSelected,
    std::shared_ptr<tf2_ros::Buffer>,
    rclcpp::Node::SharedPtr,
    QWidget * parent = nullptr);

  ~PersonWidget();


  bool selected() { return selected_; }
  void setShowId(bool show_id) { show_id_ = show_id; }

  //void updateKbVisibility() const;


protected:
  void mousePressEvent(QMouseEvent * event) override;
  void showContextMenu(const QPoint &);
  void resizeEvent(QResizeEvent *) override;
  void paintEvent(QPaintEvent *event) override;

private:

  /* move and rotate the widget based on the TF frame
   */
  void reposition();

  void updateGeometry(QPainter&);

  std::string id_;
  std::string frame_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  rclcpp::Node::SharedPtr node_;

  QTimer * timer_;

  QSvgRenderer * renderer_;
  QSvgRenderer * renderer_selected_;
  double svg_aspect_ratio_;
  QRect svg_bounds_;

  // x, y offset of the center of the head from top-left corner of the
  // widget, in pixels
  QPoint head_offset_;

  int previous_pixelPerMeter_ = 0;

  // 'physical' position in meters; x is facing the robot
  double x_ = 0., y_ = 0.;
  // orientation of the person, in radians
  double theta_ = 0.;

  bool selected_ = false;
  bool show_id_ = true;
};

}  // namespace rqt_human_radar

#endif  // RQT_HUMAN_RADAR__PERSONWIDGET_HPP_
