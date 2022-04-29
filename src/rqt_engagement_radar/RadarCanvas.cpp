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

// ROS messages
#include <hri_msgs/IdsList.h>

namespace rqt_engagement_radar {

RadarCanvas::RadarCanvas(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::RadarCanvas()) {
  ui_->setupUi(this);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, QOverload<>::of(&RadarCanvas::update));
  timer_->start(100);

  background = QImage(QSize(this->size().width(), 600), QImage::Format_RGB32);
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
  
  QPainter painter(&background);
  painter.drawImage(QPoint(0, 0), background);

  connect(ui_->fovConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::fovConeDegChanged);
  connect(ui_->fovRangeM, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::fovConeRangeChanged);
  connect(ui_->attentionConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::attentionConeDegChanged);
  connect(ui_->attentionRangeM, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::attentionConeRangeChanged);

  circleRange = 300; 
  fovRange = 400;
  attentionRange = 300;
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

void RadarCanvas::paintEvent(QPaintEvent *event){
  QPainter painter(this);
  painter.setPen(fovPen);

  painter.drawImage(QPoint(0, 0), background);

  fovRectOriginX = xOffset - fovRange;
  fovRectOriginY = yOffset - fovRange;
  fovStartAngle = -(floor(fovAmpl/2)*16 + ((fovAmpl-floor(fovAmpl/2))/100*16));
  fovSpanAngle = floor(fovAmpl)*16 + ((fovAmpl-floor(fovAmpl))/100*16); 

  QRectF fovRectangle(QPoint(fovRectOriginX, fovRectOriginY), QSize(fovRange*2, fovRange*2));
  painter.drawPie(fovRectangle, fovStartAngle, fovSpanAngle);

  painter.setPen(attentionPen);
  attentionRectOriginX = xOffset - attentionRange;
  attentionRectOriginY = yOffset - attentionRange;
  attentionStartAngle = -(floor(attentionAmpl/2)*16 + ((attentionAmpl-floor(attentionAmpl/2))/100*16));
  attentionSpanAngle = floor(attentionAmpl)*16 + ((attentionAmpl-floor(attentionAmpl))/100*16);

  QRectF attentionRectangle(QPoint(attentionRectOriginX, attentionRectOriginY), QSize(attentionRange*2, attentionRange*2));
  painter.drawPie(attentionRectangle, attentionStartAngle, attentionSpanAngle);

  // Inserting people //
  auto faces = hriListener_.getFaces();
  for(auto& face: faces){
    std::string id = face.first;
    std::string faceFrame = "face_" + id;
    tf::StampedTransform faceTrans;
    try{
      tfListener_.lookupTransform("camera_link", faceFrame, ros::Time(0), faceTrans);
      
      double personRectOriginX = xOffset + (faceTrans.getOrigin().x()*100) - (personImage.size().width()/2);
      double personRectOriginY = yOffset - (faceTrans.getOrigin().y()*100) - (personImage.size().height()/2);
      
      painter.drawImage(QPointF(personRectOriginX, personRectOriginY), personImage);
    }
    catch(tf::TransformException ex){
      ROS_WARN("%s", ex.what());
    }
  }

  QRectF range1(QPoint(xOffset - circleRange, yOffset - circleRange), QSize(2*circleRange, 2*circleRange));
  QRectF range2(QPoint(xOffset - (circleRange/2), yOffset - (circleRange/2)), QSize(circleRange, circleRange));

  painter.setPen(rangePen);
  painter.drawEllipse(range1);
  painter.drawEllipse(range2);

  painter.drawImage(QRectF(QPointF(xOffset-50, yOffset-50), QPointF(xOffset+50, yOffset+50)), robotImage);
  //painter.drawImage(QRectF(QPointF(xOffset+100, yOffset-30), QSize(70, 70)), personImage);
  //painter.drawImage(QPoint(200, 265), personImage);

  //std::cout<<personImage.size().width()<<"\t"<<personImage.size().height()<<std::endl;
}

void RadarCanvas::resizeEvent(QResizeEvent *event){
  background = QImage(QSize(this->size().width(), 600), QImage::Format_RGB32);
  background.fill(qRgb(255, 255, 255)); 
}

} /* namespace */