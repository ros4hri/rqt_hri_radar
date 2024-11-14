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
 * @file RadarCanvas.cpp
 * @brief Definition of the methods declared in RadarCanvas.hpp
 */

#include "rqt_human_radar/RadarCanvas.hpp"

#include <QAction>
#include <QCheckBox>
#include <QContextMenuEvent>
#include <QCursor>
#include <QMenu>
#include <QPainter>
#include <QPolygon>
#include <QPushButton>
#include <QRectF>
#include <QResizeEvent>
#include <QString>
#include <QStringList>
#include <QTransform>
#include <QVector>

#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>

#include "./ui_radar_tabs.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hri_msgs/msg/ids_list.hpp>
#include <hri_msgs/msg/ids_match.hpp>


using namespace std::chrono_literals;

std::vector<double> SPECIAL_ANGLES = {0, 30, 60, 90, 120, 150, 180};
const double ROBOT_SIZE = .15;   // physical size (in m) of the robot along its x-axis

namespace rqt_human_radar
{

RadarCanvas::RadarCanvas(
  QWidget * parent, Ui::RadarTabs * ui,
  rclcpp::Node::SharedPtr node)
: QWidget(parent), node_(node)
{
  setAcceptDrops(true);

  hriListener_ = hri::HRIListener::create(node_);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  timer_ = new QTimer(this);
  connect(
    timer_, &QTimer::timeout, this,
    QOverload<>::of(&RadarCanvas::update));
  timer_->start(100);

  /*
  framesTimer_ = new QTimer(this);
  connect(framesTimer_, &QTimer::timeout, this,
  QOverload<>::of(&RadarCanvas::updateFramesList)); framesTimer_->start(100);
  */

  this->ui_ = ui;

  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(
    this, &RadarCanvas::customContextMenuRequested, this,
    &RadarCanvas::showContextMenu);

  connect(
    ui_->zoomLevel, &QSlider::valueChanged, [this](int value) {
      pixelPerMeter_ = value;
      updateArcsToDraw();
      update();

      for (auto & widget : kbObjects_) {
        widget->reposition();
      }
    });

  connect(
    ui_->reloadButton, &QPushButton::clicked, this,
    &RadarCanvas::updateFramesList);

  QString currentFrameSet = ui_->refFrameComboBox->currentText();
  if (!currentFrameSet.isEmpty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Current frame set: %s",
      currentFrameSet.toStdString().c_str());
    referenceFrame_ = currentFrameSet.toStdString();
  }
  connect(
    ui_->refFrameComboBox,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    [this](int index) {
      QString frame = ui_->refFrameComboBox->itemText(index);
      if (!frame.isEmpty()) {
        RCLCPP_INFO(
          node_->get_logger(), "Current frame set: %s",
          frame.toStdString().c_str());
        referenceFrame_ = frame.toStdString();
      }
    });

  connect(
    ui_->clearObjectsBtn, &QPushButton::clicked, [this]() {
      for (auto & widget : kbObjects_) {
        widget->deleteLater();
      }
      kbObjects_.clear();
    });

  // Retrieving robot and person icons
  try {
    package_ = ament_index_cpp::get_package_share_directory("rqt_human_radar");
    robotImageFile_ = package_ + "/res/tiagopro.svg";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not find package path for 'rqt_human_radar': %s",
      e.what());
    exit(1);
  }

  robotImageFound = robotImage_.load(QString::fromStdString(robotImageFile_));

  if (!robotImageFound) {
    RCLCPP_WARN(node_->get_logger(), "Robot icon not found");
  }

  // xOffset is meant to be fixed, while yOffset depends
  // on the window size
  xOffset_ = 75;
  yOffset_ = parent->size().height() / 2;

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


  // Computation of the number of arcs to draw
  double distanceFromTopRightCorner =
    std::sqrt(
    std::pow((ui_->stackedWidget->size().width() - xOffset_), 2) +
    std::pow(yOffset_, 2));
  arcsToDraw_ = std::ceil(distanceFromTopRightCorner / pixelPerMeter_);

  // Activating mouse events
  this->setMouseTracking(true);

  // Setting to "" the name of the hovered person
  idClicked_ = "";

  // Setting callbacks for new/removed persons
  hriListener_->onTrackedPerson(
    std::bind(&RadarCanvas::onTrackedPerson, this, std::placeholders::_1));
  hriListener_->onTrackedPersonLost(
    std::bind(
      &RadarCanvas::onTrackedPersonLost,
      this, std::placeholders::_1));

  update();
}

RadarCanvas::~RadarCanvas() {}

void RadarCanvas::onTrackedPerson(hri::ConstPersonPtr person)
{
  new_persons_.push(person);
}

void RadarCanvas::onTrackedPersonLost(hri::ID id)
{
  removed_persons_.push(id);
}

bool RadarCanvas::isInFov(const QPoint & point) const
{
  if (!showFov_) {
    return false;
  }

  double angle = std::atan2(point.y() - yOffset_, point.x() - xOffset_);
  angle = angle * 180 / M_PI;
  return (angle >= -fov_ / 2) && (angle <= fov_ / 2);
}

void RadarCanvas::enableSimulation(bool state)
{
  simulationEnabled_ = state;

  if (state) {
    setContextMenuPolicy(Qt::CustomContextMenu);
  } else {
    setContextMenuPolicy(Qt::NoContextMenu);
    for (auto & widget : kbObjects_) {
      widget->deleteLater();
    }
  }
}

void RadarCanvas::paintEvent([[maybe_unused]] QPaintEvent * event)
{

  if (!new_persons_.empty()) {

    auto person = new_persons_.front();
    new_persons_.pop();
    persons_backlog_.insert(person);
  }


  // iterate over the set of persons
  for (auto it = persons_backlog_.begin(); it != persons_backlog_.end(); ) {

    auto person = *it;

    // only create a new person if it has a face or body
    // (ie, voice-only persons are not displayed)
    if (person->face() || person->body()) {


      RCLCPP_INFO(
        node_->get_logger(), "New person %s detected.",
        person->id().c_str());


      // if the person is currently anonymous, make it known

      // first, wait for the 'anonymous' flag to be set
      while (person->anonymous() == std::nullopt) {
        std::this_thread::sleep_for(10ms);
      }
      if (*person->anonymous()) {

        RCLCPP_INFO(
          node_->get_logger(), "Person %s is anonymous. Creating a new, non-anonymous person.",
          person->id().c_str());

        rclcpp::Publisher<hri_msgs::msg::IdsMatch>::SharedPtr matcher_pub_;
        matcher_pub_ = node_->create_publisher<hri_msgs::msg::IdsMatch>(
          "/humans/candidate_matches",
          10);
        hri_msgs::msg::IdsMatch match;

        // we know by construction (cf RadarCanvas) that the person has either
        // a face or a body
        if (person->face()) {
          match.id1 = person->face()->id();
          match.id1_type = hri_msgs::msg::IdsMatch::FACE;
        } else {
          match.id1 = person->body()->id();
          match.id1_type = hri_msgs::msg::IdsMatch::BODY;
        }

        match.id2 = "person_" + match.id1;
        match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
        match.confidence = 1.0;
        matcher_pub_->publish(match);

      } else {

        PersonWidget * personWidget = new PersonWidget(
          person,
          hriListener_,
          QString::fromStdString(package_ + "/res/"),
          tfBuffer_,
          node_,
          this);

        persons_[person->id()] = personWidget;
        personWidget->show();

      }

      it = persons_backlog_.erase(it);


    } else {
      it++;
    }
  }

  // mark persons without a face/body for deletion
  for (auto & person : persons_) {
    if (!person.second->isVisuallyTracked()) {
      removed_persons_.push(person.first);
    }
  }

  if (!removed_persons_.empty()) {
    auto id = removed_persons_.front();
    removed_persons_.pop();

    if (persons_.find(id) != persons_.end()) {
      persons_[id]->deleteLater();
      persons_.erase(id);
      RCLCPP_INFO(
        node_->get_logger(), "Person %s gone. Removing it.", id.c_str());
    }
  }


  double robotWidth = ROBOT_SIZE * pixelPerMeter_;
  double robotHeight = robotWidth * robotImage_.height() / robotImage_.width();

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  font_ = painter.font();


  // Ranges painting process
  painter.setPen(QPen(Qt::transparent));

  for (int arcN = arcsToDraw_; arcN > 0; arcN--) {
    // Setting circles and text parameters and support structures
    double circleRange = pixelPerMeter_ * arcN;
    double circleRectOriginX = xOffset_ - circleRange;
    double circleRectOriginY = yOffset_ - circleRange;
    double circleRectWidth = 2 * circleRange;
    double circleRectHeight = 2 * circleRange;

    QRectF circleRect(circleRectOriginX, circleRectOriginY, circleRectWidth,
      circleRectHeight);
    QPointF textPoint(xOffset_ + circleRange + 5,
      yOffset_ + (font_.pointSize() / 2));
    QString rangeDescription =
      QString::fromStdString(std::to_string(arcN) + "m");

    // Different brush, whether it is an odd or even circle
    if ((arcN % 2) == 1) {
      painter.setBrush(oddBrush_);
    } else {
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
    double distToPrint = circleRange - (0.5 * pixelPerMeter_);
    for (double angle : SPECIAL_ANGLES) {
      if (abs(angle - 90) < 1e-4) {
        continue;
      }
      angle -= 90;
      angle *= (M_PI / 180);
      double xToPrint = distToPrint * std::cos(angle) + xOffset_;
      double yToPrint = distToPrint * std::sin(angle) + yOffset_;
      if (inScreen(xToPrint, yToPrint)) {
        painter.translate(QPointF(xToPrint, yToPrint));
        painter.rotate(angle * 180 / M_PI);
        painter.drawText(
          QPointF(0, -2),
          QString::fromStdString(
            std::to_string(
              static_cast<int>(std::round(angle * 180 / M_PI)) * -1) +
            "°"));
        painter.rotate(-angle * 180 / M_PI);
        painter.translate(-QPointF(xToPrint, yToPrint));
      }
    }

    painter.setPen(QPen(Qt::transparent));
  }

  // Writing special angles values
  painter.setPen(rangePen_);

  for (double & angle : SPECIAL_ANGLES) {
    // Handling the process taking into account
    // the left handed Qt reference system
    double m = std::tan(angle / 180 * M_PI);
    double candidateYMaxWidth =
      (this->size().width() - xOffset_) / m + yOffset_;
    if (angle < 90) {
      double candidateXMaxHeight =
        m * (this->size().height() - yOffset_) + xOffset_;
      if (candidateYMaxWidth > this->size().height()) {
        painter.drawLine(
          xOffset_, yOffset_, candidateXMaxHeight,
          this->size().height());
      } else {
        painter.drawLine(
          xOffset_, yOffset_, this->size().width(),
          candidateYMaxWidth);
      }
    } else if (angle > 90) {
      double candidateXMaxHeight = -m * yOffset_ + xOffset_;
      if (candidateYMaxWidth < 0) {
        painter.drawLine(xOffset_, yOffset_, candidateXMaxHeight, 0);
      } else {
        painter.drawLine(
          xOffset_, yOffset_, this->size().width(),
          candidateYMaxWidth);
      }
    } else {
      painter.drawLine(xOffset_, yOffset_, this->size().width(), yOffset_);
    }
  }

  if (showFov_) {
    // Drawing the field of view
    painter.setPen(QPen(Qt::transparent));
    painter.setBrush(QBrush(QColor(255, 0, 0, 20)));

    double radius = pixelPerMeter_ * 3.5;
    painter.drawPie(
      xOffset_ - radius, yOffset_ - radius, radius * 2,
      radius * 2, (-fov_ / 2) * 16, fov_ * 16);
  }

  painter.setBrush(QBrush(Qt::transparent));
  painter.setPen(QPen(Qt::black));

  painter.drawImage(
    QRectF(
      QPointF(xOffset_ - robotWidth / 2, yOffset_ - robotHeight / 2),
      QPointF(xOffset_ + robotWidth / 2, yOffset_ + robotHeight / 2)),
    robotImage_);
}

void RadarCanvas::resizeEvent([[maybe_unused]] QResizeEvent * event)
{
  yOffset_ = ui_->stackedWidget->size().height() / 2;

  updateArcsToDraw();

  for (auto & widget : kbObjects_) {
    widget->reposition();
  }
}

void RadarCanvas::mousePressEvent([[maybe_unused]] QMouseEvent * event)
{
}

void RadarCanvas::showContextMenu(const QPoint & pos)
{
  QMenu contextMenu("Add objects", this);

  // (user-facing name, OWL class name, icon path)
  const std::vector<std::tuple<std::string, std::string, std::string>> OBJECTS{
    {"book", "oro:Book", package_ + "/res/icons/book-open-variant.svg"},
    {"cup", "oro:Cup", package_ + "/res/icons/cup-water.svg"},
    {"phone", "cyc:CellularTelephone", package_ + "/res/icons/cellphone.svg"},
    {"apple", "dbr:Apple", package_ + "/res/icons/food-apple.svg"},
    {"pear", "dbr:Pear", package_ + "/res/icons/food-pear.svg"},
  };

  for (const auto & object : OBJECTS) {
    std::string name, classname, icon;
    std::tie(name, classname, icon) = object;

    auto action =
      new QAction(QIcon(icon.c_str()), ("Place a " + name).c_str(), this);
    connect(
      action, &QAction::triggered, this, [name, classname, icon, this]() {
        createKbObjectWidget(name, classname, icon);
      });
    contextMenu.addAction(action);
  }

  contextMenu.exec(mapToGlobal(pos));
}

void RadarCanvas::createKbObjectWidget(
  const std::string & name,
  const std::string & classname,
  const std::string & path)
{
  KbObjectWidget * imageWidget = new KbObjectWidget(
    name, classname, QString::fromStdString(path), node_, this);
  kbObjects_.push_back(imageWidget);
  imageWidget->pxPlace(mapFromGlobal(QCursor::pos()));
  imageWidget->show();
}

void RadarCanvas::dragEnterEvent(QDragEnterEvent * event)
{
  event->acceptProposedAction();
}

void RadarCanvas::dragMoveEvent(QDragMoveEvent * event)
{
  // Accept the drag move if it stays within the widget bounds
  if (rect().contains(event->pos())) {
    event->acceptProposedAction();
  }
}

void RadarCanvas::dropEvent(QDropEvent * event)
{
  if (event->source()) {
    KbObjectWidget * draggedWidget =
      qobject_cast<KbObjectWidget *>(event->source());
    draggedWidget->pxPlace(event->pos());
    draggedWidget->show();
    event->acceptProposedAction();
  }
}

bool RadarCanvas::inScreen(double & x, double & y) const
{
  return (x > 0) && (y > 0) && (x < this->size().width()) &&
         (y < this->size().height());
}

void RadarCanvas::updateArcsToDraw()
{
  double distanceFromTopRightCorner =
    std::sqrt(
    std::pow((ui_->stackedWidget->size().width() - xOffset_), 2) +
    std::pow(yOffset_, 2));
  arcsToDraw_ = std::ceil(distanceFromTopRightCorner / pixelPerMeter_);
}

void RadarCanvas::updateFramesList()
{
  std::vector<std::string> framesAvailable;
  tfBuffer_->_getFrameStrings(framesAvailable);
  QStringList framesAvailableQ;
  QString prevValue = ui_->refFrameComboBox->currentText();

  ui_->refFrameComboBox->clear();
  int indexPrevValue = -1;
  int index = 0;

  for (auto & frame : framesAvailable) {
    framesAvailableQ << QString::fromStdString(frame);
    if (frame == prevValue.toStdString()) {
      indexPrevValue = index;
    }
    index++;
  }

  ui_->refFrameComboBox->insertItems(0, framesAvailableQ);

  if (indexPrevValue != -1) {
    ui_->refFrameComboBox->setCurrentIndex(indexPrevValue);
  }
}

}  // namespace rqt_human_radar
