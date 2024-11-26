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

#include <QGraphicsView>
#include <QMouseEvent>

namespace rqt_human_radar
{

class InteractiveView : public QGraphicsView
{
  Q_OBJECT

public:
  explicit InteractiveView(QWidget * parent = nullptr);

  void resetView();

protected:
  void wheelEvent(QWheelEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

private:
  double scaleFactor_ = 1.0;
  double metersPerPixel_ = 0.01;  // Default: 1 meter = 100 pixels
};

}  // namespace rqt_human_radar
