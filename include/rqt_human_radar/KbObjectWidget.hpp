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

#ifndef RQT_HUMAN_RADAR__KBOBJECTWIDGET_HPP_
#define RQT_HUMAN_RADAR__KBOBJECTWIDGET_HPP_

#include <memory>
#include <string>

#include <QSvgWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class KbObjectWidget : public QSvgWidget {
    Q_OBJECT
public:
  KbObjectWidget(
        const std::string &classname,
        const QString &file,
        rclcpp::Node::SharedPtr,
        QWidget *parent = nullptr);

  ~KbObjectWidget();
protected:
  void mousePressEvent(QMouseEvent *event) override;
  void showContextMenu(const QPoint &);

private:

  std::string classname_;
  std::string id_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kb_add_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kb_remove_pub_;

};

#endif  // RQT_HUMAN_RADAR__KBOBJECTWIDGET_HPP_
