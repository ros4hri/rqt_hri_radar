#include <iostream>
#include <cmath>

#include "rqt_engagement_radar/RadarCanvas.hpp"

#include <QPainter>
#include <QRectF>
#include <cmath>
#include <QString>
#include <QTransform>
#include <QResizeEvent>
#include <QGraphicsView>

// ROS Utilities
#include <ros/package.h>
#include <ros/console.h>

// ROS messages
#include <hri_msgs/IdsList.h>

#include "ui_radar_tabs.h"

namespace rqt_engagement_radar {

RadarCanvas::RadarCanvas(QWidget *parent, Ui::RadarTabs* ui_) :
    QWidget(parent){
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, QOverload<>::of(&RadarCanvas::update));
  timer_->start(100);

  this->ui_ = ui_;

  connect(ui_->fovConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::fovConeDegChanged);
  connect(ui_->fovRangeM, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::fovConeRangeChanged);
  connect(ui_->attentionConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::attentionConeDegChanged);
  connect(ui_->attentionRangeM, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::attentionConeRangeChanged);

  background = QImage(QSize(this->size().width(), this->size().height()), QImage::Format_RGB32);
  background.fill(qRgb(255, 255, 255));   

  // Retrieving robot and person icons
  package = ros::package::getPath("rqt_engagement_radar");
  robotImageFile = package + "/img/ARI_icon.png";
  personImageFile = package + "/img/Person_icon.png";
  robotImageFound = robotImage.load(QString::fromStdString(robotImageFile));
  personImageFound = personImage.load(QString::fromStdString(personImageFile));

  if (!robotImageFound){
    ROS_WARN("Robot icon not found");
  }
  else{
    // Rotating image
    QTransform tr;
    tr.rotate(-50);
    robotImage = robotImage.transformed(tr);
  }

  if(!personImageFound){
    ROS_WARN("Person icon not found");
  }else{
    personImage = personImage.scaledToHeight(70);
  }

  fovRange = 400;
  attentionRange = 300;
  xOffset = 50; 
  yOffset = ui_->tab->size().height()/2; 

  QColor lightRed(255, 235, 235);
  QColor lightGreen(235, 255, 235);

  fovPen = QPen(lightGreen, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  attentionPen = QPen(lightRed, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  rangePen = QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

  fovBrush = QBrush(lightGreen);
  attentionBrush = QBrush(lightRed);

  versor_.vector.x = 1.0;
  versor_.vector.y = 0.0;
  versor_.vector.z = 0.0;

  arcsToDraw = std::floor(ui_->tab->size().width()/100);

  update();
}

RadarCanvas::~RadarCanvas() {
}

void RadarCanvas::paintEvent(QPaintEvent *event){
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  versor_.header.stamp = ros::Time(0);

  painter.drawImage(QPoint(0, 0), background);

  fovRectOriginX = xOffset - fovRange;
  fovRectOriginY = yOffset - fovRange;
  fovStartAngle = -(floor(fovAmpl/2)*16 + ((fovAmpl-floor(fovAmpl/2))/100*16));
  fovSpanAngle = floor(fovAmpl)*16 + ((fovAmpl-floor(fovAmpl))/100*16); 

  painter.setPen(fovPen);
  painter.setBrush(fovBrush);
  QRectF fovRectangle(QPoint(fovRectOriginX, fovRectOriginY), QSize(fovRange*2, fovRange*2));
  painter.drawPie(fovRectangle, fovStartAngle, fovSpanAngle);

  painter.setPen(attentionPen);
  painter.setBrush(attentionBrush);
  attentionRectOriginX = xOffset - attentionRange;
  attentionRectOriginY = yOffset - attentionRange;
  attentionStartAngle = -(floor(attentionAmpl/2)*16 + ((attentionAmpl-floor(attentionAmpl/2))/100*16));
  attentionSpanAngle = floor(attentionAmpl)*16 + ((attentionAmpl-floor(attentionAmpl))/100*16);

  QRectF attentionRectangle(QPoint(attentionRectOriginX, attentionRectOriginY), QSize(attentionRange*2, attentionRange*2));
  painter.drawPie(attentionRectangle, attentionStartAngle, attentionSpanAngle);

  painter.setBrush(QBrush(Qt::transparent));

  // Inserting people //
  auto faces = hriListener_.getFaces();
  for(auto& face: faces){
    geometry_msgs::Vector3Stamped rotatedVersor;

    std::string id = face.first;
    std::string faceFrame = "face_" + id;
    versor_.header.frame_id = faceFrame;
    tf::StampedTransform faceTrans;

    try{
      tfListener_.lookupTransform("camera_link", faceFrame, ros::Time(0), faceTrans);
      tfListener_.transformVector("camera_link", versor_, rotatedVersor);
      double theta = std::atan2(-(rotatedVersor.vector.y), -(rotatedVersor.vector.x));

      QTransform tr;
      tr.rotate(-(theta*180)/M_PI);
      QImage personRotated = personImage.transformed(tr);
      
      double personRectOriginX = xOffset + (faceTrans.getOrigin().x()*100) - (personImage.size().width()/2);
      double personRectOriginY = yOffset - (faceTrans.getOrigin().y()*100) - (personImage.size().height()/2);
      
      painter.drawImage(QPointF(personRectOriginX, personRectOriginY), personRotated);
    }
    catch(tf::TransformException ex){
      ROS_WARN("%s", ex.what());
    }
  }

  // Ranges painting process

  painter.setPen(rangePen);

  for(int arcN = 1; arcN <= arcsToDraw; arcN++){
    double circleRange = 100*arcN;
    double circleRectOriginX = xOffset - circleRange;
    double circleRectOriginY = yOffset - circleRange;
    double circleRectWidth = 2*circleRange;
    double circleRectHeight = 2*circleRange;

    QRectF circleRect(circleRectOriginX, circleRectOriginY, circleRectWidth, circleRectHeight);

    if(circleRange <= (background.size().height()/2)){
      painter.drawEllipse(circleRect);
    } else{
      double theta = std::asin(background.size().height()/2/circleRange);
      theta *= 180/M_PI; // rad to deg
      theta *= 16; // deg to QtDeg 
      painter.drawArc(circleRect, -theta, 2*theta);
    }
  }

  painter.drawImage(QRectF(QPointF(xOffset-50, yOffset-50), QPointF(xOffset+50, yOffset+50)), robotImage);
}

void RadarCanvas::resizeEvent(QResizeEvent *event){
  yOffset = ui_->tab->size().height()/2;

  arcsToDraw = std::floor((ui_->tab->size().width())/100);

  background = QImage(QSize(ui_->tab->size().width(), ui_->tab->size().height()), QImage::Format_RGB32);
  background.fill(qRgb(255, 255, 255)); 
}

void RadarCanvas::fovConeDegChanged(){
  fovAmpl = ui_->fovConeDeg->value();
   
  update();
}

void RadarCanvas::fovConeRangeChanged(){
  fovRange = ui_->fovRangeM->value()*100; 
  update();
}

void RadarCanvas::attentionConeDegChanged(){
  attentionAmpl = ui_->attentionConeDeg->value();
  update();
}

void RadarCanvas::attentionConeRangeChanged(){
  attentionRange = ui_->attentionRangeM->value()*100; 
  update();
}

void RadarCanvas::showEvent(QShowEvent *event){
  ui_->radarCanvas->setGeometry(QRect(0, 30, ui_->tab->size().width(), ui_->tab->size().height()));
}

} /* namespace */