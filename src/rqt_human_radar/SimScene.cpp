// Copyright 2024 PAL Robotics S.L.
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

#include "rqt_human_radar/SimScene.hpp"
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QPainter>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hri_msgs/msg/ids_list.hpp>
#include <hri_msgs/msg/ids_match.hpp>

#include "rqt_human_radar/LocalObjectItem.hpp"
#include "rqt_human_radar/LocalPersonItem.hpp"
#include "rqt_human_radar/RemoteObjectItem.hpp"

using namespace std::chrono_literals;

namespace rqt_human_radar
{

SimScene::SimScene(rclcpp::Node::SharedPtr node)
: node_(node)
{
  setSceneRect(
    -5 * pixelsPerMeter, -5 * pixelsPerMeter, 15 * pixelsPerMeter,
    10 * pixelsPerMeter);            // Set the size of the scene

  // Retrieving the resource path, with our icons, etc
  try {
    package_ = ament_index_cpp::get_package_share_directory("rqt_human_radar");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not find package path for 'rqt_human_radar': %s",
      e.what());
    exit(1);
  }

  // configure the HRI listener
  hriListener_ = hri::HRIListener::create(node_);

  // Setting callbacks for new/removed persons
  hriListener_->onTrackedPerson(
    std::bind(&SimScene::onTrackedPerson, this, std::placeholders::_1));
  hriListener_->onTrackedPersonLost(
    std::bind(&SimScene::onTrackedPersonLost, this, std::placeholders::_1));

  auto robotItem = new PersonItem(
    node_, "myself", "oro:Robot",
    package_ + "/res/tiagopro.svg");
  robotItem->setFovColor(QColor(0, 200, 100, 1));
  robotItem->setPos(0, 0);
  addItem(robotItem);
  robotItem->setPhysicalWidth(0.2);  // 20cm


  updateTimer_ = new QTimer(this);
  connect(updateTimer_, &QTimer::timeout, this, &SimScene::updatePersons);
  updateTimer_->start(50);  // milliseconds
}

void SimScene::enableSimulation(bool state)
{
  simulationEnabled_ = state;

  if (!state) {
    clearPersons();
    clearObjects();
  }
}
void SimScene::clearPersons()
{
  QList<QGraphicsItem *> itemsInScene = items();
  QList<QGraphicsItem *> itemsToRemove;

  std::copy_if(
    itemsInScene.begin(), itemsInScene.end(),
    std::back_inserter(itemsToRemove), [](QGraphicsItem * item) {
      return dynamic_cast<LocalPersonItem *>(item);
    });

  std::for_each(
    itemsToRemove.begin(), itemsToRemove.end(),
    [this](QGraphicsItem * item) {
      removeItem(item);
      delete item;
    });
}

void SimScene::clearObjects()
{
  QList<QGraphicsItem *> itemsInScene = items();
  QList<QGraphicsItem *> itemsToRemove;

  std::copy_if(
    itemsInScene.begin(), itemsInScene.end(),
    std::back_inserter(itemsToRemove), [](QGraphicsItem * item) {
      return dynamic_cast<LocalObjectItem *>(item);
    });

  std::for_each(
    itemsToRemove.begin(), itemsToRemove.end(),
    [this](QGraphicsItem * item) {
      removeItem(item);
      delete item;
    });
}

void SimScene::drawBackground(
  QPainter * painter,
  [[maybe_unused]] const QRectF & rect)
{
  // Color definition
  QColor lightGrey(232, 232, 233);
  QColor lighterGrey(237, 238, 239);
  QColor midGrey(175, 175, 175);

  // Brush definition. Used to paint the background
  auto evenBrush_ = QBrush(lightGrey);
  auto oddBrush_ = QBrush(lighterGrey);
  auto rangePen_ = QPen(midGrey, 5, Qt::SolidLine);

  auto font_ = painter->font();
  font_.setPointSize(0.1 * pixelsPerMeter);
  painter->setFont(font_);

  int radius = std::ceil((sceneRect().width() / 2) / pixelsPerMeter);

  while (radius > 0) {
    QString rangeDescription =
      QString::fromStdString(std::to_string(radius) + "m");
    QPointF textPoint((radius + 0.1) * pixelsPerMeter, (font_.pointSize() / 2));

    if ((radius % 2) == 1) {
      painter->setBrush(oddBrush_);
    } else {
      painter->setBrush(evenBrush_);
    }

    painter->drawEllipse(
      QPointF(0, 0), radius * pixelsPerMeter,
      radius * pixelsPerMeter);

    painter->setPen(rangePen_);
    painter->translate(textPoint);
    painter->rotate(90);
    painter->drawText(QPoint(0, 0), rangeDescription);
    painter->rotate(-90);
    painter->translate(-textPoint);

    radius -= 1;
  }

  // draw X axis
  painter->setPen(rangePen_);
  painter->drawLine(-sceneRect().width() / 2, 0, sceneRect().width() / 2, 0);
  painter->setPen(QPen(Qt::red, 10, Qt::SolidLine));
  painter->drawLine(0, 0, 500, 0);

  // draw Y axis
  painter->setPen(rangePen_);
  painter->drawLine(0, -sceneRect().height() / 2, 0, sceneRect().height() / 2);
  painter->setPen(QPen(Qt::green, 10, Qt::SolidLine));
  painter->drawLine(0, 0, 0, -500);  // Y axis is inverted
}

void SimScene::contextMenuEvent(QGraphicsSceneContextMenuEvent * event)
{
  if (!simulationEnabled_) {
    return;
  }

  // Check if an item is under the cursor
  if (itemAt(event->scenePos(), QTransform())) {
    // Let the item's context menu handle the event
    QGraphicsScene::contextMenuEvent(event);
    return;
  }

  // Otherwise, show the scene's context menu
  QMenu menu;

  // first, add an entry to add a human
  auto human_action = new QAction(
    QIcon((package_ + "/res/icons/Agent.svg").c_str()), "Add human", this);
  connect(
    human_action, &QAction::triggered, this, [event, this]() {
      auto localItem = new LocalPersonItem(node_, package_, "base_link");

      // Set its position to the mouse click location
      localItem->setPos(event->scenePos());
      addItem(localItem);
    });

  menu.addAction(human_action);
  menu.addSeparator();

  // then, add objects
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
      action, &QAction::triggered, this,
      [event, name, classname, icon, this]() {
        auto localItem = new LocalObjectItem(
          node_, name, icon, classname,
          "base_link", true);

        // Set its position to the mouse click location
        localItem->setPos(event->scenePos());

        // Add the item to the scene
        addItem(localItem);
      });
    menu.addAction(action);
  }

  menu.exec(event->screenPos());
}

void SimScene::updatePersons()
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
          node_->get_logger(),
          "Person %s is anonymous. Creating a new, non-anonymous person.",
          person->id().c_str());

        rclcpp::Publisher<hri_msgs::msg::IdsMatch>::SharedPtr matcher_pub_;
        matcher_pub_ = node_->create_publisher<hri_msgs::msg::IdsMatch>(
          "/humans/candidate_matches", 10);
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
        auto personItem =
          new RemotePersonItem(node_, person, package_, "base_link");
        persons_[person->id()] = personItem;
        addItem(personItem);
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
      removeItem(persons_[id]);
      delete persons_[id];
      persons_.erase(id);
      RCLCPP_INFO(
        node_->get_logger(), "Person %s gone. Removing it.",
        id.c_str());
    }
  }
}

void SimScene::onTrackedPerson(hri::ConstPersonPtr person)
{
  new_persons_.push(person);
}

void SimScene::onTrackedPersonLost(hri::ID id) {removed_persons_.push(id);}
}  // namespace rqt_human_radar
