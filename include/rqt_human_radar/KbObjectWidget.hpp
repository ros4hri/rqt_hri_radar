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

#include <QSvgWidget>

class KbObjectWidget : public QSvgWidget {
    Q_OBJECT
public:
  KbObjectWidget(const QString &file,
                                QWidget *parent = nullptr);

protected:
  void mousePressEvent(QMouseEvent *event) override;
};

#endif  // RQT_HUMAN_RADAR__KBOBJECTWIDGET_HPP_
