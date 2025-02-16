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

#include <QGraphicsView>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QSpinBox>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>


#include "rqt_human_radar/InteractiveView.hpp"
#include "rqt_human_radar/SimScene.hpp"
#include "rqt_human_radar/SimUi.hpp"

#include "./ui_radar_tabs.h"

namespace rqt_human_radar
{

SimUi::SimUi(QWidget * parent, rclcpp::Node::SharedPtr node)
: QWidget(parent), ui_(new Ui::RadarTabs()), node_(node)
{
  ui_->setupUi(this);

  auto scene = new SimScene(node);
  auto simView = new InteractiveView(this);
  simView->setScene(scene);

  ui_->radarCanvas = simView;

  simView->resetView();

  if (ui_->objectsSimCheckbox->isChecked()) {
    ui_->simHelpLabel->show();
    ui_->clearPersonsBtn->show();
    ui_->clearObjectsBtn->show();
    ui_->downloadTplBtn->setEnabled(true);
    ui_->loadMapBtn->setEnabled(true);
    scene->enableSimulation(true);
  } else {
    ui_->simHelpLabel->hide();
    ui_->clearPersonsBtn->hide();
    ui_->clearObjectsBtn->hide();
    ui_->downloadTplBtn->setEnabled(false);
    ui_->loadMapBtn->setEnabled(false);
    scene->enableSimulation(false);
  }

  // Initial value of showFov
  scene->showFov(ui_->fovCheckbox->isChecked());
  scene->setFov(ui_->fov->value());

  // Initial value of showIDs
  scene->showIds(ui_->idsCheckbox->isChecked());

  connect(
    ui_->clearPersonsBtn, &QPushButton::clicked,
    [ = ]() {scene->clearPersons();});

  connect(
    ui_->clearObjectsBtn, &QPushButton::clicked,
    [ = ]() {scene->clearObjects();});

  connect(
    ui_->resetViewBtn, &QPushButton::clicked,
    [ = ]() {simView->resetView();});

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
    ui_->loadMapBtn, &QPushButton::clicked, [ = ]() {
      auto fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open SVG environment"), "", "SVG environment (*.svg)");
      environment_loader_.loadMap(node_, scene, fileName.toStdString());
    });

  connect(
    ui_->downloadTplBtn, &QPushButton::clicked, [ = ]() {
      auto tpl = QString::fromStdString(scene->package_path() + "/res/map.svg");

      auto destinationFilePath = QFileDialog::getSaveFileName(
        this,
        tr("Save SVG template as"), QDir::homePath(), "SVG environment (*.svg)");

      if (destinationFilePath.isEmpty()) {
        return;
      }
      // Copy the file to the selected location
      QFile sourceFile(tpl);
      QFile destinationFile(destinationFilePath);
      if (sourceFile.exists()) {
        if (!destinationFile.exists() || destinationFile.remove() ) {
          if (!QFile::copy(tpl, destinationFilePath)) {
            QMessageBox::critical(nullptr, "Error", "Failed to save the template.");
          }
        }
      } else {
        QMessageBox::critical(nullptr, "Error", "Source template file does not exist.");
      }
    });

  connect(
    ui_->fovCheckbox, &QCheckBox::stateChanged, [ = ](int state) {
      ui_->fov->setEnabled(state);
      scene->showFov(state);
    });
  connect(
    ui_->fov,
    static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
    [ = ](int value) {scene->setFov(value);});

  connect(
    ui_->idsCheckbox, &QCheckBox::stateChanged,
    [ = ](int state) {scene->showIds(state);});

  connect(
    ui_->objectsSimCheckbox, &QCheckBox::stateChanged, [ = ](int state) {
      scene->enableSimulation(state);
      if (state) {
        ui_->simHelpLabel->show();
        ui_->clearPersonsBtn->show();
        ui_->clearObjectsBtn->show();
        ui_->downloadTplBtn->setEnabled(true);
        ui_->loadMapBtn->setEnabled(true);
      } else {
        ui_->simHelpLabel->hide();
        ui_->clearPersonsBtn->hide();
        ui_->clearObjectsBtn->hide();
        ui_->downloadTplBtn->setEnabled(false);
        ui_->loadMapBtn->setEnabled(false);
      }
    });
}

SimUi::~SimUi() {delete ui_;}

void SimUi::showRadarCanvas()
{
  if (ui_->stackedWidget->currentIndex() == 0) {
    ui_->radarCanvas->show();
  } else {
    ui_->radarCanvas->hide();
  }
}

void SimUi::resizeEvent([[maybe_unused]] QResizeEvent * event)
{
  ui_->radarCanvas->setGeometry(
    ui_->stackedWidget->geometry().adjusted(1, 1, -1, -1));
}

void SimUi::showEvent([[maybe_unused]] QShowEvent * event)
{
  ui_->radarCanvas->setGeometry(
    ui_->stackedWidget->geometry().adjusted(1, 1, -1, -1));
  ui_->stackedWidget->setCurrentIndex(0);
}

}  // namespace rqt_human_radar
