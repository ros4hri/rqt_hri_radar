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

/**
 * @file RadarCanvas.hpp
 * @brief Defines the class for the RadarCanvas widget.
 *
 * The widget where the radar background
 * and icons for the rqt-humans-radar are painted
 * taking into account the preferences expressed
 * by the user in the settings tab.
 */
#ifndef RQT_HUMAN_RADAR__RADARCANVAS_HPP_
#define RQT_HUMAN_RADAR__RADARCANVAS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QTimer>
#include <QDockWidget>
#include <QStringList>
#include <QWidget>
#include <QtSvg/QSvgRenderer>
#include <QtSvg/QSvgWidget>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QPen>
#include <QImage>
#include <QSize>
#include <QLine>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <hri/hri.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "rqt_human_radar/KbObjectWidget.hpp"
#include "rqt_human_radar/PersonWidget.hpp"

#include "rqt_human_radar/concurrent_queue.hpp"

namespace Ui
{
class RadarTabs;
}

namespace rqt_human_radar
{

class RadarCanvas : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  RadarCanvas(QWidget * parent, Ui::RadarTabs * ui, rclcpp::Node::SharedPtr node);

  /**
   * @brief Destructor
   */
  virtual ~RadarCanvas();

  void onTrackedPerson(hri::ConstPersonPtr person);

  void onTrackedPersonLost(hri::ID id);

  void updateFramesList();

public slots:
  /**
   * @brief Reading the user preference about showing or not people ID.
   *
   * The preference is expressed in settings,
   * through a tick-box. Currently, person ID = <person_id>
   */
  void showIds(bool state)
  {
    showIds_ = state;
    update();
  }

  /**
   * @brief Reading the user preference about showing or not the field of view.
   *
   * The preference is expressed in settings.
   */
  void showFov(bool state)
  {
    showFov_ = state;
    update();
  }

  bool hasFov() const
  {
    return showFov_;

    for (auto & widget : kbObjects_) {
      widget->updateKbVisibility();
    }
  }

  void setFov(int value)
  {
    fov_ = value;
    update();

    for (auto & widget : kbObjects_) {
      widget->updateKbVisibility();
    }
  }

  std::tuple<double, double> getOffset() const
  {
    return {xOffset_, yOffset_};
  }

  int getPixelPerMeter() const
  {
    return pixelPerMeter_;
  }

  std::optional<std::string> getReferenceFrame() const
  {
    return referenceFrame_;
  }

  bool isInFov(const QPoint & pos) const;

  void enableSimulation(bool state);

protected:
  /**
   * @brief overriding the paintEvent virtual function.
   *
   * Inherited from QWidget. Paints the entire radar canvas.
   */
  void paintEvent(QPaintEvent * event) override;
  /**
   * @brief overriding the resizeEvent virtual function.
   *
   * Inherited from QWidget. Updates some painting parameters
   * according to the new size of the window.
   */
  void resizeEvent(QResizeEvent * event) override;
  /**
   * @brief overriding the mousePressEvent virtual function
   *
   * Inherited from QWidget. Stores the id of the person
   * icon the user has clicked on, if any.
   */
  void mousePressEvent(QMouseEvent * event) override;

private:
  void showContextMenu(const QPoint &);
  void createKbObjectWidget(const std::string &, const std::string &, const std::string &);

  void dragEnterEvent(QDragEnterEvent * event) override;
  void dragMoveEvent(QDragMoveEvent * event) override;
  void dropEvent(QDropEvent * event) override;

  /**
   * @brief returns whether or not a point is inside the canvas.
   */
  bool inScreen(double & x, double & y) const;
  /**
   * @brief updates the number of arcs to draw.
   *
   * The numbers of arcs to draw is defined by the current
   * size of the canvas.
   */
  void updateArcsToDraw();

  QTimer * timer_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<hri::HRIListener> hriListener_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

  // Drawing and painting objects
  QPen rangePen_;
  QBrush oddBrush_, evenBrush_;
  QFont font_, anglesFont_;

  // Stores the image being drawn
  QImage robotImage_;
  bool robotImageFound;
  std::string package_, robotImageFile_;
  std::map<std::string, PersonWidget*> persons_;
  concurrent_queue<hri::ConstPersonPtr> new_persons_;
  concurrent_queue<std::string> removed_persons_;

  // store here persons that have been detected, but do not have yet
  // a face/body associated. They will be drawn as soon as they have
  // a face/body.
  std::set<hri::ConstPersonPtr> persons_backlog_;

  int pixelPerMeter_;

  int arcsToDraw_;

  // Radar drawing components
  double xOffset_, yOffset_;

  // New stuff to avoid using ui
  QWidget * widget_;
  Ui::RadarTabs * ui_;

  // ID clicked with mouse
  std::string idClicked_;

  // Reference frame
  std::optional<std::string> referenceFrame_;

  std::vector<KbObjectWidget *> kbObjects_;


  bool showFov_;
  int fov_;
  bool showIds_;
  bool simulationEnabled_;
};

}  // namespace rqt_human_radar
#endif  // RQT_HUMAN_RADAR__RADARCANVAS_HPP_
