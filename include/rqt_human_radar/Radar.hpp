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
 * @file Radar.hpp
 * @brief Defines the class for the Radar widget.
 */

#ifndef RQT_HUMAN_RADAR__RADAR_HPP_
#define RQT_HUMAN_RADAR__RADAR_HPP_

#include <QWidget>
#include <rqt_gui_cpp/plugin.h>
#include <unistd.h>

namespace rqt_human_radar
{

class Radar;

class Radar : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  Radar();
  /**
   *  @brief Destructor
   */
  virtual ~Radar();

  /**
   * @brief template function for rqt plugins
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  /**
   * @brief template function for rqt plugins
   */
  virtual void shutdownPlugin();
  /**
   * @brief template function for rqt plugins
   */
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;
  /**
   * @brief template function for rqt plugins
   */
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

private:
  QWidget * widget_;
};

}  // namespace rqt_human_radar

#endif  // RQT_HUMAN_RADAR__RADAR_HPP_
