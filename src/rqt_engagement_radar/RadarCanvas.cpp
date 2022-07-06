#include <iostream>
#include <cmath>

#include "rqt_engagement_radar/RadarCanvas.hpp"

#include <QPainter>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QTransform>
#include <QResizeEvent>
#include <QGraphicsView>
#include <QPolygon>

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

  pixelPerMeter = 150; 

  QColor lightRed(255, 235, 235);
  QColor lightGreen(235, 255, 235);
  QColor lightGrey(232, 232, 233);
  QColor lighterGrey(237, 238, 239);

  fovPen = QPen(lightGreen, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  attentionPen = QPen(lightRed, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  rangePen = QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

  fovBrush = QBrush(lightGreen);
  attentionBrush = QBrush(lightRed);
  evenBrush = QBrush(lightGrey);
  oddBrush = QBrush(lighterGrey);

  versor_.vector.x = 1.0;
  versor_.vector.y = 0.0;
  versor_.vector.z = 0.0;

  //computing the number of arcs to draw
  
  double distanceFromTopRightCorner = std::sqrt(std::pow((ui_->tab->size().width() - xOffset), 2) + std::pow(yOffset, 2));
  arcsToDraw = std::ceil(distanceFromTopRightCorner/pixelPerMeter);

  // Activating mouse events
  this->setMouseTracking(true);

  // Setting to "none" the name of the hovered person
  idHovered = "none";

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

  // Ranges painting process

  painter.setPen(QPen(Qt::transparent));

  for(int arcN = arcsToDraw; arcN > 0; arcN--){
    double circleRange = pixelPerMeter*arcN;
    double circleRectOriginX = xOffset - circleRange;
    double circleRectOriginY = yOffset - circleRange;
    double circleRectWidth = 2*circleRange;
    double circleRectHeight = 2*circleRange;

    QRectF circleRect(circleRectOriginX, circleRectOriginY, circleRectWidth, circleRectHeight);

    if ((arcN%2) == 1)
      painter.setBrush(oddBrush);
    else
      painter.setBrush(evenBrush);

    painter.drawEllipse(circleRect);
  }

  painter.setBrush(QBrush(Qt::transparent));
  painter.setPen(QPen(Qt::black));

  // Inserting people //
  auto faces = hriListener_.getFaces();
  peoplePosition.clear();
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

      // Left handed rotation

      double rotatedWidthX = (personImage.size().width()/2*std::cos(theta));
      double rotatedWidthY = (personImage.size().width()/2*-std::sin(theta));

      double rotatedHeightX = (personImage.size().height()/2*std::sin(theta));
      double rotatedHeightY = (personImage.size().height()/2*std::cos(theta));

      // Computing vertices of the rotated rectangle
      
      double personRectOriginX = xOffset + (faceTrans.getOrigin().x()*pixelPerMeter) - rotatedHeightX - rotatedWidthX;
      double personRectOriginY = yOffset - (faceTrans.getOrigin().y()*pixelPerMeter) - rotatedHeightY - rotatedWidthY;

      double personRectRightCornerX = xOffset + (faceTrans.getOrigin().x()*pixelPerMeter) + rotatedHeightX + rotatedWidthX;
      double personRectRightCornerY = yOffset - (faceTrans.getOrigin().y()*pixelPerMeter) + rotatedHeightY + rotatedWidthY;

      double personRectBottomLeftX = xOffset + (faceTrans.getOrigin().x()*pixelPerMeter) + rotatedHeightX - rotatedWidthX;
      double personRectBottomLeftY = yOffset - (faceTrans.getOrigin().y()*pixelPerMeter) + rotatedHeightY - rotatedWidthY;

      double personRectTopRightX = xOffset + (faceTrans.getOrigin().x()*pixelPerMeter) - rotatedHeightX + rotatedWidthX;
      double personRectTopRightY = yOffset - (faceTrans.getOrigin().y()*pixelPerMeter) - rotatedHeightY + rotatedWidthY;

      QPointF topLeftCorner(personRectOriginX, personRectOriginY);
      QPointF topLeftCornerAnti(-personRectOriginX, -personRectOriginY);
      QPointF bottomRightCorner(personRectRightCornerX, personRectRightCornerY);
      QPointF bottomLeftCorner(personRectBottomLeftX, personRectBottomLeftY);
      QPointF topRightCorner(personRectTopRightX, personRectTopRightY);

      // Image translation and rotation (via QPainter methods)

      painter.translate(topLeftCorner);
      painter.rotate(-(theta*180)/M_PI);
      painter.drawImage(QPointF(0, 0), personImage);
      painter.rotate((theta*180)/M_PI);
      painter.translate(topLeftCornerAnti);

      // Storing the person's containing polygon
      // A rectangle object can not represent rotated rectangles

      QVector<QPoint> points;
      points.append(topLeftCorner.toPoint());
      points.append(bottomLeftCorner.toPoint());
      points.append(bottomRightCorner.toPoint());
      points.append(topRightCorner.toPoint());

      peoplePosition.insert(std::pair<std::string, QPolygon>(id, QPolygon(points)));

      if(id==idHovered){
        QString identificator = QString::fromStdString("id: "+idHovered);
        painter.drawText(bottomRightCorner, identificator);
      }
    }
    catch(tf::TransformException ex){
      ROS_WARN("%s", ex.what());
    }
  }

  painter.drawImage(QRectF(QPointF(xOffset-50, yOffset-50), QPointF(xOffset+50, yOffset+50)), robotImage);
}

void RadarCanvas::resizeEvent(QResizeEvent *event){
  yOffset = ui_->tab->size().height()/2;

  double distanceFromTopRightCorner = std::sqrt(std::pow((ui_->tab->size().width() - xOffset), 2) + std::pow(yOffset, 2));
  arcsToDraw = std::ceil(distanceFromTopRightCorner/pixelPerMeter);

  background = QImage(QSize(ui_->tab->size().width(), ui_->tab->size().height()), QImage::Format_RGB32);
  background.fill(qRgb(255, 255, 255)); 
}

void RadarCanvas::fovConeDegChanged(){
  fovAmpl = ui_->fovConeDeg->value();
  update();
}

void RadarCanvas::fovConeRangeChanged(){
  fovRange = ui_->fovRangeM->value()*pixelPerMeter; 
  update();
}

void RadarCanvas::attentionConeDegChanged(){
  attentionAmpl = ui_->attentionConeDeg->value();
  update();
}

void RadarCanvas::attentionConeRangeChanged(){
  attentionRange = ui_->attentionRangeM->value()*pixelPerMeter; 
  update();
}

void RadarCanvas::showEvent(QShowEvent *event){
  ui_->radarCanvas->setGeometry(QRect(0, 30, ui_->tab->size().width(), ui_->tab->size().height()));
}

void RadarCanvas::mouseMoveEvent(QMouseEvent *event){
  for(auto& elem: peoplePosition){
    if(elem.second.containsPoint(QPoint(event->x(), event->y()), Qt::OddEvenFill)){
      idHovered = elem.first;
      return; // No more than one hovered person at a time
    }
  }
  idHovered = "none";
}

} /* namespace */