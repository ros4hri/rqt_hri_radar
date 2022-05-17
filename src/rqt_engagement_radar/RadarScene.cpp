#include <iostream>
#include <cmath>

#include "rqt_engagement_radar/RadarScene.hpp"
#include "rqt_engagement_radar/RadarCanvas.hpp"

#include "ui_radar_scene.h"

#include <QPainter>
#include <QRectF>
#include <cmath>
#include <QString>
#include <QTransform>
#include <QResizeEvent>
#include <QSizePolicy>

// ROS Utilities
#include <ros/package.h>
#include <ros/console.h>

// ROS messages
#include <hri_msgs/IdsList.h>

namespace rqt_engagement_radar {

RadarScene::RadarScene(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::RadarScene()) {
  ui_->setupUi(this); 
  ui_->radarCanvas = new RadarCanvas(this, ui_);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, QOverload<>::of(&RadarScene::update));
  timer_->start(100);   

  ui_->verticalWidget->setGeometry(QRect(0, 0, this->size().width()-ui_->radarCanvas->size().width(), this->size().height()));
}

RadarScene::~RadarScene() {
  //confirmClose();

  delete ui_;
}

void RadarScene::resizeEvent(QResizeEvent *event){
  ui_->horizontalWidget->setGeometry(QRect(0, 0, event->size().width(), event->size().height()));
  ui_->radarCanvas->setGeometry(QRect(0, 0, event->size().width()-ui_->verticalWidget->size().width(), event->size().height()));
}

void RadarScene::showEvent(QShowEvent *event){
  ui_->radarCanvas->setGeometry(QRect(0, 0, this->size().width()-ui_->verticalWidget->size().width(), this->size().height()));
}

} /* namespace */