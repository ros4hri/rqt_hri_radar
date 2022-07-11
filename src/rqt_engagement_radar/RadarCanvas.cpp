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
#include <QtSvg/QSvgWidget>
#include <QtSvg/QSvgRenderer>
#include <QFont>

// ROS Utilities
#include <ros/package.h>
#include <ros/console.h>

// ROS messages
#include <hri_msgs/IdsList.h>

#include "ui_radar_tabs.h"

std::vector<double> SPECIAL_ANGLES = {0, 30, 60, 90, 120, 150, 180};
double SVG_SIZE_RATIO = 1.4857;
double SVG_PERSON_WIDTH = 70;

namespace rqt_engagement_radar {

RadarCanvas::RadarCanvas(QWidget *parent, Ui::RadarTabs* ui_) :
    QWidget(parent){
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, QOverload<>::of(&RadarCanvas::update));
  timer_->start(100);

  this->ui_ = ui_;

  connect(ui_->ppmSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &RadarCanvas::updatePixelPerMeter);

  background = QImage(QSize(this->size().width(), this->size().height()), QImage::Format_RGB32);
  background.fill(qRgb(255, 255, 255));   

  // Retrieving robot and person icons
  package = ros::package::getPath("rqt_engagement_radar");
  robotImageFile = package + "/img/ARI_icon.png";
  personSvgFile = package + "/img/adult_standing_disengaging.svg";
  robotImageFound = robotImage.load(QString::fromStdString(robotImageFile));

  svgRendererInitialized = svgRenderer.load(QString::fromStdString(personSvgFile));

  // Svg rendered initialization

  if (!robotImageFound){
    ROS_WARN("Robot icon not found");
  }

  // TODO: removing this import and only using
  // the svg file. Setting as a variable the size
  // for the person and storing as a constant the 
  // h/w ratio for the svg file.

  if(!svgRendererInitialized){
    ROS_WARN("Person icon not found");
  }

  xOffset = 50; 
  yOffset = ui_->tab->size().height()/2;

  pixelPerMeter = 300; 

  QColor lightRed(255, 235, 235);
  QColor lightGreen(235, 255, 235);
  QColor lightGrey(232, 232, 233);
  QColor lighterGrey(237, 238, 239);
  QColor midGrey(175, 175, 175);

  evenBrush = QBrush(lightGrey);
  oddBrush = QBrush(lighterGrey);
  rangePen = QPen(midGrey);

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
  font = painter.font();
  
  // Leaving this here for possible discussions about angles/range sizing
  // It could be removed if we go for one single font size/style
  anglesFont = font; 

  versor_.header.stamp = ros::Time(0);

  painter.drawImage(QPoint(0, 0), background);

  // Ranges painting process

  painter.setPen(QPen(Qt::transparent));

  font.setPointSize(font.pointSize());
  anglesFont.setPointSize(font.pointSize());

  for(int arcN = arcsToDraw; arcN > 0; arcN--){
    double circleRange = pixelPerMeter*arcN;
    double circleRectOriginX = xOffset - circleRange;
    double circleRectOriginY = yOffset - circleRange;
    double circleRectWidth = 2*circleRange;
    double circleRectHeight = 2*circleRange;

    QRectF circleRect(circleRectOriginX, circleRectOriginY, circleRectWidth, circleRectHeight);
    QPointF textPoint(xOffset + circleRange + 5, yOffset + (font.pointSize()/2));
    QString rangeDescription = QString::fromStdString(std::to_string(arcN)+"m");

    if ((arcN%2) == 1){
      painter.setBrush(oddBrush);
    }
    else{
      painter.setBrush(evenBrush);
    }

    painter.setFont(font);

    painter.drawEllipse(circleRect);
    painter.setPen(rangePen);
    painter.translate(textPoint);
    painter.rotate(90);
    painter.drawText(QPoint(0, 0), rangeDescription); 
    painter.rotate(-90);
    painter.translate(-textPoint);

    // Printing angle values

    painter.setFont(anglesFont);

    double distToPrint = circleRange - (0.5*pixelPerMeter);
    for(double angle: SPECIAL_ANGLES){
      if (angle == 90)
        continue;
      angle -= 90;
      angle *= (M_PI/180);
      double xToPrint = distToPrint*std::cos(angle) + xOffset;
      double yToPrint = distToPrint*std::sin(angle) + yOffset;
      if (inScreen(xToPrint, yToPrint)){
        painter.translate(QPointF(xToPrint, yToPrint));
        painter.rotate(angle*180/M_PI);
        painter.drawText(QPointF(0, -2), QString::fromStdString(std::to_string(int(std::round(angle*180/M_PI))*-1)+"°")); // 2 extra pixels for readibility reasons
        painter.rotate(-angle*180/M_PI);
        painter.translate(-QPointF(xToPrint, yToPrint));
      }
    }

    painter.setPen(QPen(Qt::transparent));
  }

  // Drawing special angles

  painter.setPen(rangePen);

  for(double& angle: SPECIAL_ANGLES){
    double m = std::tan(angle/180*M_PI);
    double candidateYMaxWidth = (this->size().width() - xOffset)/m + yOffset;
    if(angle < 90){
      double candidateXMaxHeight = m*(this->size().height() - yOffset) + xOffset;
      if(candidateYMaxWidth > this->size().height())
        painter.drawLine(xOffset, yOffset, candidateXMaxHeight, this->size().height());
      else
        painter.drawLine(xOffset, yOffset, this->size().width(), candidateYMaxWidth); 
    }
    else if(angle > 90){
      double candidateXMaxHeight = -m*yOffset + xOffset; // at this point, max height is represented by 0 on y axis
      if(candidateYMaxWidth < 0)
        painter.drawLine(xOffset, yOffset, candidateXMaxHeight, 0);
      else
        painter.drawLine(xOffset, yOffset, this->size().width(), candidateYMaxWidth); 
    }else{
      painter.drawLine(xOffset, yOffset, this->size().width(), yOffset);
    }
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
      theta += M_PI/2;

      // Left handed rotation

      // Half width

      double rotatedWidthX = (SVG_PERSON_WIDTH/2*std::cos(theta));
      double rotatedWidthY = (SVG_PERSON_WIDTH/2*-std::sin(theta));

      // Half height

      double rotatedHeightX = (SVG_PERSON_WIDTH/SVG_SIZE_RATIO/2*std::sin(theta));
      double rotatedHeightY = (SVG_PERSON_WIDTH/SVG_SIZE_RATIO/2*std::cos(theta));

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
      QPointF bottomRightCorner(personRectRightCornerX, personRectRightCornerY);
      QPointF bottomLeftCorner(personRectBottomLeftX, personRectBottomLeftY);
      QPointF topRightCorner(personRectTopRightX, personRectTopRightY);

      // Image translation and rotation (via QPainter methods)

      painter.translate(topLeftCorner);
      painter.rotate(-(theta*180)/M_PI);
      svgRenderer.render(&painter, QRectF(QPointF(0, 0), QPointF(SVG_PERSON_WIDTH, SVG_PERSON_WIDTH/SVG_SIZE_RATIO)));
      painter.rotate((theta*180)/M_PI);
      painter.translate(-topLeftCorner);

      // Storing the person's containing polygon
      // A rectangle object can not represent rotated rectangles

      QVector<QPoint> points;
      points.append(topLeftCorner.toPoint());
      points.append(bottomLeftCorner.toPoint());
      points.append(bottomRightCorner.toPoint());
      points.append(topRightCorner.toPoint());

      peoplePosition.insert(std::pair<std::string, QPolygon>(id, QPolygon(points)));

      QString identificator = QString::fromStdString(id);
      painter.drawText(bottomRightCorner, identificator);
    }
    catch(tf::TransformException ex){
      ROS_WARN("%s", ex.what());
    }
  }

  painter.drawImage(QRectF(QPointF(xOffset-50, yOffset-50), QPointF(xOffset+50, yOffset+50)), robotImage);
}

void RadarCanvas::resizeEvent(QResizeEvent *event){
  yOffset = ui_->tab->size().height()/2;

  updateArcsToDraw();

  background = QImage(QSize(ui_->tab->size().width(), ui_->tab->size().height()), QImage::Format_RGB32);
  background.fill(qRgb(255, 255, 255)); 
}

void RadarCanvas::updatePixelPerMeter(){
  pixelPerMeter = ui_->ppmSpinBox->value();
  updateArcsToDraw(); 
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

bool RadarCanvas::inScreen(double& x, double& y) const{
  return (x > 0) && (y > 0) && (x < this->size().width()) && (y < this->size().height());
}

void RadarCanvas::updateArcsToDraw(){
  double distanceFromTopRightCorner = std::sqrt(std::pow((ui_->tab->size().width() - xOffset), 2) + std::pow(yOffset, 2));
  arcsToDraw = std::ceil(distanceFromTopRightCorner/pixelPerMeter);
}

} /* namespace */