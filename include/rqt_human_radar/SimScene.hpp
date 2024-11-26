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

#pragma once

#include <QGraphicsScene>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <hri/hri.hpp>

#include "rqt_human_radar/RemotePersonItem.hpp"
#include "rqt_human_radar/concurrent_queue.hpp"

namespace rqt_human_radar
{
class SimScene : public QGraphicsScene
{
  Q_OBJECT

public:
  // rendering resolution for the scene
  static constexpr double pixelsPerMeter = 1000.;

  explicit SimScene(rclcpp::Node::SharedPtr node);

  void addObject(const std::string & id);

public slots:
  void enableSimulation(bool state);

  void showFov(bool show)
  {
    showFov_ = show;
    update();
  }
  void setFov(double fov)
  {
    fov_ = fov;
    update();
  }
  void showIds(bool show)
  {
    showIds_ = show;
    update();
  }

  void clearPersons();
  void clearObjects();

protected:
  void drawBackground(QPainter * painter, const QRectF & rect) override;
  void contextMenuEvent(QGraphicsSceneContextMenuEvent * event) override;

private:
  void updatePersons();

  void onTrackedPerson(hri::ConstPersonPtr person);

  void onTrackedPersonLost(hri::ID id);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<hri::HRIListener> hriListener_;

  std::string package_;

  bool simulationEnabled_ = false;
  bool showFov_ = true;
  double fov_ = 120.0;
  bool showIds_ = true;

  std::map<std::string, RemotePersonItem *> persons_;
  concurrent_queue<hri::ConstPersonPtr> new_persons_;
  concurrent_queue<std::string> removed_persons_;

  // store here persons that have been detected, but do not have yet
  // a face/body associated. They will be drawn as soon as they have
  // a face/body.
  std::set<hri::ConstPersonPtr> persons_backlog_;

  QTimer * updateTimer_;
};
}  // namespace rqt_human_radar
