/**
 * @file RadarCanvas.hpp
 * @brief Definition of the methods declared in RadarScene.hpp
 */

#include "rqt_human_radar/RadarScene.hpp"
#include "rqt_human_radar/RadarCanvas.hpp"

#include "ui_radar_tabs.h"

#include <QResizeEvent>
#include <QSizePolicy>

// ROS Utilities
#include <ros/package.h>
#include <ros/console.h>

// ROS messages
#include <hri_msgs/IdsList.h>

namespace rqt_human_radar {

RadarScene::RadarScene(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::RadarTabs()) {
  ui_->setupUi(this); 
  ui_->radarCanvas = new RadarCanvas(this, ui_);

  connect(ui_->tabWidget, QOverload<int>::of(&QTabWidget::currentChanged), this, &RadarScene::showRadarCanvas);
}

RadarScene::~RadarScene() {
  delete ui_;
}

void RadarScene::showRadarCanvas(){
  if (ui_->tabWidget->currentIndex() == 0){
    ui_->radarCanvas->show();
  }else{
    ui_->radarCanvas->hide();
  }
}

void RadarScene::resizeEvent(QResizeEvent *event){
  ui_->tabWidget->setGeometry(QRect(0, 0, event->size().width(), event->size().height()));
  ui_->tab->setGeometry(QRect(0, 0, event->size().width(), event->size().height()));
  ui_->radarCanvas->setGeometry(QRect(0, 30, ui_->tab->size().width(), ui_->tab->size().height()));
}

void RadarScene::showEvent(QShowEvent *event){
  ui_->tabWidget->setGeometry(QRect(0, 0, this->size().width(), this->size().height()));
  ui_->radarCanvas->setGeometry(QRect(0, 30, ui_->tab->size().width(), this->size().height()));
  ui_->tabWidget->setCurrentIndex(0);
}

} /* namespace */