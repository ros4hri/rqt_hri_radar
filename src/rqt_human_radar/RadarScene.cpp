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
 * @brief Definition of the methods declared in RadarScene.hpp
 */

#include <QResizeEvent>
#include <QSizePolicy>

#include "rqt_human_radar/RadarScene.hpp"
#include "rqt_human_radar/RadarCanvas.hpp"

#include "./ui_radar_tabs.h"

namespace rqt_human_radar
{

RadarScene::RadarScene(QWidget * parent, rclcpp::Node::SharedPtr node)
: QWidget(parent), ui_(new Ui::RadarTabs()), node_(node)
{
  ui_->setupUi(this);
  ui_->radarCanvas = new RadarCanvas(this, ui_, node_);

  connect(
    ui_->tabWidget, QOverload<int>::of(&QTabWidget::currentChanged), this,
    &RadarScene::showRadarCanvas);
}

RadarScene::~RadarScene() {delete ui_;}

void RadarScene::showRadarCanvas()
{
  if (ui_->tabWidget->currentIndex() == 0) {
    ui_->radarCanvas->show();
  } else {
    ui_->radarCanvas->hide();
  }
}

void RadarScene::resizeEvent(QResizeEvent * event)
{
  ui_->tabWidget->setGeometry(
    QRect(0, 0, event->size().width(), event->size().height()));
  ui_->tab->setGeometry(
    QRect(0, 0, event->size().width(), event->size().height()));
  ui_->radarCanvas->setGeometry(
    QRect(0, 30, ui_->tab->size().width(), ui_->tab->size().height()));
}

void RadarScene::showEvent([[maybe_unused]] QShowEvent * event)
{
  ui_->tabWidget->setGeometry(
    QRect(0, 0, this->size().width(), this->size().height()));
  ui_->radarCanvas->setGeometry(
    QRect(0, 30, ui_->tab->size().width(), this->size().height()));
  ui_->tabWidget->setCurrentIndex(0);
}

}  // namespace rqt_human_radar
