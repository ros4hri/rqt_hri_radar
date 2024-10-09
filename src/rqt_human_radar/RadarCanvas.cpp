/**
 * @file RadarCanvas.cpp
 * @brief Definition of the methods declared in RadarCanvas.hpp
 */
#include <sstream>
#include <cmath>
#include <functional>

#include "rqt_human_radar/RadarCanvas.hpp"

#include <QPainter>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QTransform>
#include <QResizeEvent>
#include <QPolygon>
#include <QCheckBox>
#include <QStringList>
#include <QPushButton>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For transforming geometry_msgs types

// ROS Utilities
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS messages
#include <hri_msgs/msg/ids_list.hpp>

#include "ui_radar_tabs.h"

std::vector<double> SPECIAL_ANGLES = {0, 30, 60, 90, 120, 150, 180};
const double SVG_SIZE_RATIO = 1.4857;
const double SVG_PERSON_WIDTH = 70;

namespace rqt_human_radar {

RadarCanvas::RadarCanvas(QWidget *parent, Ui::RadarTabs* ui, rclcpp::Node::SharedPtr node) :    QWidget(parent),
    node_(node)
    {

    hriListener_ = hri::HRIListener::create(node_);

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, QOverload<>::of(&RadarCanvas::update));
  timer_->start(100);

  /*
  framesTimer_ = new QTimer(this);
  connect(framesTimer_, &QTimer::timeout, this, QOverload<>::of(&RadarCanvas::updateFramesList));
  framesTimer_->start(100);
  */

  this->ui_ = ui;

  connect(ui_->ppmSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &RadarCanvas::updatePixelPerMeter);
  connect(ui_->idCheckbox, QOverload<int>::of(&QCheckBox::stateChanged), this, &RadarCanvas::showId);
  connect(ui_->reloadButton, &QPushButton::clicked, this, &RadarCanvas::updateFramesList);

  // Retrieving robot and person icons
  try {
    package_ = ament_index_cpp::get_package_share_directory("rqt_human_radar");
    robotImageFile_ = package_ + "/res/ARI_icon.png";
    personSvgFile_ = package_ + "/res/adult_standing_disengaging.svg";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find package path for 'rqt_human_radar': %s", e.what());
    exit(1);
  }

  robotImageFound = robotImage_.load(QString::fromStdString(robotImageFile_));

  svgRendererInitialized_ = svgRenderer_.load(QString::fromStdString(personSvgFile_));

  if (!robotImageFound){
    RCLCPP_WARN(node_->get_logger(), "Robot icon not found");
  }

  if(!svgRendererInitialized_){
    RCLCPP_WARN(node_->get_logger(), "Person icon not found");
  }

  // xOffset is meant to be fixed, while yOffset depends
  // on the window size
  xOffset_ = 50; 
  yOffset_ = parent->size().height()/2;

  // Initial value, it is possible to modify it through settings
  pixelPerMeter_ = 300; 

  // Color definition
  QColor lightGrey(232, 232, 233);
  QColor lighterGrey(237, 238, 239);
  QColor midGrey(175, 175, 175);

  // Brush definition. Used to paint the background
  evenBrush_ = QBrush(lightGrey);
  oddBrush_ = QBrush(lighterGrey);
  rangePen_ = QPen(midGrey);

  // Initialization of a constant versor
  // which is later used to orient person icons. 
  versor_.vector.x = 1.0;
  versor_.vector.y = 0.0;
  versor_.vector.z = 0.0;

  // Computation of the number of arcs to draw
  double distanceFromTopRightCorner = 
    std::sqrt(std::pow((ui_->tab->size().width() - xOffset_), 2) + std::pow(yOffset_, 2));
  arcsToDraw_ = std::ceil(distanceFromTopRightCorner/pixelPerMeter_);

  // Activating mouse events
  this->setMouseTracking(true);

  // Setting to "" the name of the hovered person
  idClicked_ = "";

  // Reading the value representing whether we should display people ids or not
  showIdValue_ = ui_->idCheckbox->checkState();

  // Setting callbacks for new/removed bodies
  hriListener_->onBody(std::bind(&RadarCanvas::onBody, this, std::placeholders::_1));
  hriListener_->onBodyLost(std::bind(&RadarCanvas::onBodyLost, this, std::placeholders::_1));

  update();
}

RadarCanvas::~RadarCanvas() {
}

void RadarCanvas::onBody(hri::ConstBodyPtr body){
    bodies_.push_back(body->id());
}

void RadarCanvas::onBodyLost(hri::ID id){
  auto body = std::find(bodies_.begin(), bodies_.end(), id);
  bodies_.erase(body);
}

void RadarCanvas::paintEvent(QPaintEvent *event){
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  font_ = painter.font();

  versor_.header.stamp = rclcpp::Time();

  // Ranges painting process
  painter.setPen(QPen(Qt::transparent));

  for(int arcN = arcsToDraw_; arcN > 0; arcN--){

    // Setting circles and text parameters and support structures
    double circleRange = pixelPerMeter_*arcN;
    double circleRectOriginX = xOffset_ - circleRange;
    double circleRectOriginY = yOffset_ - circleRange;
    double circleRectWidth = 2*circleRange;
    double circleRectHeight = 2*circleRange;

    QRectF circleRect(
      circleRectOriginX, circleRectOriginY, circleRectWidth, circleRectHeight);
    QPointF textPoint(
      xOffset_ + circleRange + 5, yOffset_ + (font_.pointSize()/2));
    QString rangeDescription = 
      QString::fromStdString(std::to_string(arcN)+"m");

    // Different brush, whether it is an odd or even circle
    if ((arcN%2) == 1){
      painter.setBrush(oddBrush_);
    }
    else{
      painter.setBrush(evenBrush_);
    }

    painter.drawEllipse(circleRect);
    painter.setPen(rangePen_);
    painter.translate(textPoint);
    painter.rotate(90);
    painter.drawText(QPoint(0, 0), rangeDescription); 
    painter.rotate(-90);
    painter.translate(-textPoint);

    // Printing angle values
    double distToPrint = circleRange - (0.5*pixelPerMeter_);
    for(double angle: SPECIAL_ANGLES){
      if (abs(angle - 90) < 1e-4)
        continue;
      angle -= 90;
      angle *= (M_PI/180);
      double xToPrint = distToPrint*std::cos(angle) + xOffset_;
      double yToPrint = distToPrint*std::sin(angle) + yOffset_;
      if (inScreen(xToPrint, yToPrint)){
        painter.translate(QPointF(xToPrint, yToPrint));
        painter.rotate(angle*180/M_PI);
        painter.drawText(
          QPointF(0, -2), 
          QString::fromStdString(
            std::to_string(int(std::round(angle*180/M_PI))*-1)+"°"));
        painter.rotate(-angle*180/M_PI);
        painter.translate(-QPointF(xToPrint, yToPrint));
      }
    }

    painter.setPen(QPen(Qt::transparent));
  }

  // Writing special angles values
  painter.setPen(rangePen_);

  for(double& angle: SPECIAL_ANGLES){
    // Handling the process taking into account 
    // the left handed Qt reference system
    double m = std::tan(angle/180*M_PI);
    double candidateYMaxWidth = (this->size().width() - xOffset_)/m + yOffset_;
    if(angle < 90){
      double candidateXMaxHeight = 
        m*(this->size().height() - yOffset_) + xOffset_;
      if(candidateYMaxWidth > this->size().height())
        painter.drawLine(
          xOffset_, yOffset_, candidateXMaxHeight, this->size().height());
      else
        painter.drawLine(
          xOffset_, yOffset_, this->size().width(), candidateYMaxWidth); 
    }
    else if(angle > 90){
      double candidateXMaxHeight = -m*yOffset_ + xOffset_;
      if(candidateYMaxWidth < 0)
        painter.drawLine(xOffset_, yOffset_, candidateXMaxHeight, 0);
      else
        painter.drawLine(
          xOffset_, yOffset_, this->size().width(), candidateYMaxWidth); 
    }else{
      painter.drawLine(xOffset_, yOffset_, this->size().width(), yOffset_);
    }
  }

  painter.setBrush(QBrush(Qt::transparent));
  painter.setPen(QPen(Qt::black));

  // Drawing people. Using body_<body_id> frames
  // to define people's position and orientation
  auto bodies = hriListener_->getBodies();
  peoplePosition_.clear();
  for(auto& body: bodies){
    geometry_msgs::msg::Vector3Stamped rotatedVersor;

    std::string id = body.first;
    std::string bodyFrame = "body_" + id;
    versor_.header.frame_id = bodyFrame;
    geometry_msgs::msg::TransformStamped bodyTrans;

    QString currentFrameSet = ui_->refFrameComboBox->currentText();
    if (currentFrameSet.isEmpty()){
      referenceFrame_.reset();
      updateFramesList();
    }else{
      referenceFrame_ = currentFrameSet.toStdString();
    }

    if (referenceFrame_){
      try{
          bodyTrans = tfBuffer_->lookupTransform(
            *referenceFrame_, bodyFrame, rclcpp::Time(0), rclcpp::Duration(1, 0));  // 1-second timeout;
          tf2::doTransform(versor_, rotatedVersor, bodyTrans);

          double distance = 
            std::sqrt(std::pow(bodyTrans.transform.translation.x, 2) 
              + std::pow(bodyTrans.transform.translation.y, 2));
          double theta = 
            std::atan2(-(rotatedVersor.vector.y), -(rotatedVersor.vector.x));
          theta += M_PI/2;
    
          // Left-handed rotation of the rectangle containing the person's icon
          // to draw
    
          // The rectangle used here is initially managed as it was
          // centered in (0, 0) and subsequently translated. 
          // When referencing to "width" and "height", it is actually
          // half of the width and height of the rectangle containing
          // the person's icon.
    
          double rotatedWidthX = (SVG_PERSON_WIDTH/2*std::cos(theta));
          double rotatedWidthY = (SVG_PERSON_WIDTH/2*-std::sin(theta));
    
          double rotatedHeightX = 
            (SVG_PERSON_WIDTH/SVG_SIZE_RATIO/2*std::sin(theta));
          double rotatedHeightY = 
            (SVG_PERSON_WIDTH/SVG_SIZE_RATIO/2*std::cos(theta));
    
          // Computing vertices of the rotated rectangle
          
          double personRectTopLeftX = 
            xOffset_ 
            + (bodyTrans.transform.translation.x*pixelPerMeter_) 
            - rotatedHeightX 
            - rotatedWidthX;
          double personRectTopLeftY = 
            yOffset_ 
            - (bodyTrans.transform.translation.y*pixelPerMeter_) 
            - rotatedHeightY 
            - rotatedWidthY;
    
          double personRectBottomRightX = 
            xOffset_ 
            + (bodyTrans.transform.translation.x*pixelPerMeter_) 
            + rotatedHeightX 
            + rotatedWidthX;
          double personRectBottomRightY = 
            yOffset_ 
            - (bodyTrans.transform.translation.y*pixelPerMeter_) 
            + rotatedHeightY 
            + rotatedWidthY;
    
          double personRectBottomLeftX = 
            xOffset_ 
            + (bodyTrans.transform.translation.x*pixelPerMeter_) 
            + rotatedHeightX 
            - rotatedWidthX;
          double personRectBottomLeftY = 
            yOffset_ 
            - (bodyTrans.transform.translation.y*pixelPerMeter_) 
            + rotatedHeightY 
            - rotatedWidthY;
    
          double personRectTopRightX = 
            xOffset_ 
            + (bodyTrans.transform.translation.x*pixelPerMeter_) 
            - rotatedHeightX 
            + rotatedWidthX;
          double personRectTopRightY = 
            yOffset_ 
            - (bodyTrans.transform.translation.y*pixelPerMeter_) 
            - rotatedHeightY 
            + rotatedWidthY;
    
          QPointF topLeftCorner(personRectTopLeftX, personRectTopLeftY);
          QPointF bottomRightCorner(personRectBottomRightX, personRectBottomRightY);
          QPointF bottomLeftCorner(personRectBottomLeftX, personRectBottomLeftY);
          QPointF topRightCorner(personRectTopRightX, personRectTopRightY);
    
          // Image translation and rotation (via QPainter methods)
    
          painter.translate(topLeftCorner);
          painter.rotate(-(theta*180)/M_PI);
          svgRenderer_.render(
            &painter, QRectF(QPointF(0, 0), QPointF(SVG_PERSON_WIDTH, SVG_PERSON_WIDTH/SVG_SIZE_RATIO)));
          painter.rotate((theta*180)/M_PI);
          painter.translate(-topLeftCorner);
    
          // Storing the person's containing polygon
          // A rectangle object can not represent rotated rectangles
    
          QVector<QPoint> points;
          points.append(topLeftCorner.toPoint());
          points.append(bottomLeftCorner.toPoint());
          points.append(bottomRightCorner.toPoint());
          points.append(topRightCorner.toPoint());
    
          peoplePosition_.insert(
            std::pair<std::string, QPolygon>(id, QPolygon(points)));
    
          // Showing people ID when option selected in 
          // radar settings
          // Showing people distance when clicking on them
          QString identificator = QString::fromStdString(id);
          if(showIdValue_ == Qt::Checked)
            painter.drawText(bottomRightCorner, identificator);
          if(idClicked_ == id){
            std::ostringstream distanceStream;
            distanceStream << std::fixed << std::setprecision(2) << distance;
            std::string distanceString = distanceStream.str();
            QString distanceInfo = 
              QString::fromStdString("Distance: " + distanceString);
            painter.drawText(bottomRightCorner, identificator);
            painter.drawText(bottomLeftCorner, distanceInfo);
          }
        }
        catch(tf2::TransformException &ex){
          RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
        }
      }
  }

  painter.drawImage(
    QRectF(
      QPointF(xOffset_-50, yOffset_-50), QPointF(xOffset_+50, yOffset_+50)), robotImage_);
}

void RadarCanvas::resizeEvent(QResizeEvent *event){
  yOffset_ = ui_->tab->size().height()/2;

  updateArcsToDraw();
}

void RadarCanvas::updatePixelPerMeter(){
  pixelPerMeter_ = ui_->ppmSpinBox->value();
  updateArcsToDraw(); 
  update();
}

void RadarCanvas::showId(){
  showIdValue_ = ui_->idCheckbox->checkState();
  update();
}

void RadarCanvas::mousePressEvent(QMouseEvent *event){
  for(auto& elem: peoplePosition_){
    if(elem.second.containsPoint(QPoint(event->x(), event->y()), Qt::OddEvenFill)){
      idClicked_ = (idClicked_ != elem.first)?elem.first:"";
      return; // No more than one clicked person at a time
    }
  }
}

bool RadarCanvas::inScreen(double& x, double& y) const{
  return (x > 0) && (y > 0) && (x < this->size().width()) && (y < this->size().height());
}

void RadarCanvas::updateArcsToDraw(){
  double distanceFromTopRightCorner = 
    std::sqrt(std::pow((ui_->tab->size().width() - xOffset_), 2) + std::pow(yOffset_, 2));
  arcsToDraw_ = std::ceil(distanceFromTopRightCorner/pixelPerMeter_);
}

void RadarCanvas::updateFramesList(){
  std::vector<std::string> framesAvailable;
  tfBuffer_->_getFrameStrings(framesAvailable);
  QStringList framesAvailableQ;
  QString prevValue = ui_->refFrameComboBox->currentText();

  ui_->refFrameComboBox->clear();
  int indexPrevValue = -1;
  int index = 0;

  for (auto &frame: framesAvailable){
    framesAvailableQ << QString::fromStdString(frame);
    if (frame == prevValue.toStdString()){
      indexPrevValue = index;
    }
    index++;
  }

  ui_->refFrameComboBox->insertItems(0, framesAvailableQ);

  if (indexPrevValue != -1){
    ui_->refFrameComboBox->setCurrentIndex(indexPrevValue);
  }
}

} /* namespace */
