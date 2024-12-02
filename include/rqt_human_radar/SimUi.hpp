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

#pragma once

#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include "rqt_human_radar/EnvironmentLoader.hpp"

namespace Ui
{
class RadarTabs;
}

namespace rqt_human_radar
{

class SimUi : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  explicit SimUi(QWidget * parent = 0, rclcpp::Node::SharedPtr node = nullptr);
  /**
   * @brief Destructor
   */
  virtual ~SimUi();

public slots:
  /**
   * @brief function managing the radar canvas.
   *
   * function called when current tab goes
   * from settings to radar.
   */
  void showRadarCanvas();

protected:
  /**
   * @brief function managing a resizing event.
   */
  void resizeEvent(QResizeEvent * event) override;
  /**
   * @brief function managing the window pop-up.
   */
  void showEvent(QShowEvent * event) override;

private:
  Ui::RadarTabs * ui_;

  // ROS 2 node
  rclcpp::Node::SharedPtr node_;

  EnvironmentLoader environment_loader_;

  QTimer timer_;
};

}  // namespace rqt_human_radar
