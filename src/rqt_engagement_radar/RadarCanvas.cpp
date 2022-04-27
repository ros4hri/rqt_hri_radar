#include <iostream>

#include "rqt_engagement_radar/RadarCanvas.hpp"

#include "ui_radar_canvas.h"

#include <QPainter>
#include <QRectF>
#include <cmath>
#include <QString>
#include <QTransform>

// ROS Utilities
#include <ros/package.h>
#include <ros/console.h>

namespace rqt_engagement_radar {

RadarCanvas::RadarCanvas(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::RadarCanvas()) {
  ui_->setupUi(this);
  
  image = QImage(QSize(800, 600), QImage::Format_RGB32);
  image.fill(qRgb(255, 255, 255));

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
  
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), image);

  connect(ui_->fovConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::fovConeDegChanged);
  connect(ui_->attentionConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::attentionConeDegChanged);

  detectorLength = 300; // TODO: allowing the user to set this value
  xOffset = 50; // Random value
  yOffset = 300; // Image length (in this case 600) / 2 ==> 300

  fovPen = QPen(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  attentionPen = QPen(Qt::red, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  rangePen = QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

  update();
}

RadarCanvas::~RadarCanvas() {
  //confirmClose();

  delete ui_;
}

void RadarCanvas::fovConeDegChanged(){
  fovAmpl = ui_->fovConeDeg->value(); 
  update();
}

void RadarCanvas::attentionConeDegChanged(){
  attentionAmpl = ui_->attentionConeDeg->value();
  update();
}

void RadarCanvas::paintEvent(QPaintEvent *event){
  QPainter painter(this);
  painter.setPen(fovPen);

  image = QImage(QSize(800, 600), QImage::Format_RGB32);
  image.fill(qRgb(255, 255, 255));

  painter.drawImage(QPoint(0, 0), image);

  fovRectOriginX = xOffset - detectorLength;
  fovRectOriginY = yOffset - detectorLength;
  fovStartAngle = -(floor(fovAmpl/2)*16 + ((fovAmpl-floor(fovAmpl/2))/100*16));
  fovSpanAngle = floor(fovAmpl)*16 + ((fovAmpl-floor(fovAmpl))/100*16); 

  QRectF fovRectangle(QPoint(fovRectOriginX, fovRectOriginY), QSize(detectorLength*2, detectorLength*2));
  painter.drawPie(fovRectangle, fovStartAngle, fovSpanAngle);

  painter.setPen(attentionPen);
  attentionStartAngle = -(floor(attentionAmpl/2)*16 + ((attentionAmpl-floor(attentionAmpl/2))/100*16));
  attentionSpanAngle = floor(attentionAmpl)*16 + ((attentionAmpl-floor(attentionAmpl))/100*16);

  painter.drawPie(fovRectangle, attentionStartAngle, attentionSpanAngle);

  QRectF range1(QPoint(fovRectOriginX + detectorLength/2, fovRectOriginY + detectorLength/2), QSize(detectorLength, detectorLength));
  QRectF range2(QPoint(fovRectOriginX + detectorLength/4, fovRectOriginY + detectorLength/4), QSize(detectorLength*3/2, detectorLength*3/2));

  painter.setPen(rangePen);
  painter.drawEllipse(range1);
  painter.drawEllipse(range2);

  painter.drawImage(QRectF(QPointF(xOffset-50, yOffset-50), QPointF(xOffset+50, yOffset+50)), robotImage);
  //painter.drawImage(QRectF(QPointF(xOffset+100, yOffset-30), QSize(70, 70)), personImage);
  painter.drawImage(QPoint(200, 265), personImage);

}

} /* namespace */