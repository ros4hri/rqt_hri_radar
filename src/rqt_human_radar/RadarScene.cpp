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
#include <QSpinBox>

#include "rqt_human_radar/RadarScene.hpp"
#include "rqt_human_radar/RadarCanvas.hpp"

#include "./ui_radar_tabs.h"

namespace rqt_human_radar
{

RadarScene::RadarScene(QWidget * parent, rclcpp::Node::SharedPtr node)
: QWidget(parent), ui_(new Ui::RadarTabs()), node_(node)
{
  ui_->setupUi(this);
  RadarCanvas * radarCanvas = new RadarCanvas(this, ui_, node_);
  ui_->radarCanvas = radarCanvas;

  // Initial value of showFov
  radarCanvas->showFov(ui_->fovCheckbox->isChecked());
  radarCanvas->setFov(ui_->fov->value());

  // Initial value of showIDs
  radarCanvas->showIds(ui_->idsCheckbox->isChecked());

  connect(
    ui_->settingsBtn, &QPushButton::clicked, [ = ]() {
      ui_->stackedWidget->setCurrentIndex(1);
      ui_->radarCanvas->hide();
    });
  connect(
    ui_->doneSettingsBtn, &QPushButton::clicked, [ = ]() {
      ui_->stackedWidget->setCurrentIndex(0);
      ui_->radarCanvas->show();
    });

  connect(
    ui_->zoomIn, &QPushButton::clicked, [ = ]() {
      ui_->zoomLevel->setValue(ui_->zoomLevel->value() + ui_->zoomLevel->singleStep());
    });

  connect(
    ui_->zoomOut, &QPushButton::clicked, [ = ]() {
      ui_->zoomLevel->setValue(ui_->zoomLevel->value() - ui_->zoomLevel->singleStep());
    });

  connect(
    ui_->fovCheckbox, &QCheckBox::stateChanged, [ = ](int state) {
      ui_->fov->setEnabled(state);
      radarCanvas->showFov(state);
    });
  connect(
    ui_->fov, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [ = ](int value) {
      radarCanvas->setFov(value);
    });
  connect(
    ui_->idsCheckbox, &QCheckBox::stateChanged, [ = ](int state) {
      radarCanvas->showIds(state);
    });
}

RadarScene::~RadarScene() {delete ui_;}

void RadarScene::showRadarCanvas()
{
  if (ui_->stackedWidget->currentIndex() == 0) {
    ui_->radarCanvas->show();
  } else {
    ui_->radarCanvas->hide();
  }
}

void RadarScene::resizeEvent([[maybe_unused]] QResizeEvent * event)
{
  ui_->radarCanvas->setGeometry(
    ui_->stackedWidget->geometry().adjusted(1, 1, -1, -1));
}

void RadarScene::showEvent([[maybe_unused]] QShowEvent * event)
{
  ui_->radarCanvas->setGeometry(
    ui_->stackedWidget->geometry().adjusted(1, 1, -1, -1));
  ui_->stackedWidget->setCurrentIndex(0);
}

}  // namespace rqt_human_radar
