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
 * @file Radar.cpp
 * @brief Definition of the methods declared in Radar.hpp
 */

#include "rqt_human_radar/Radar.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rqt_human_radar/SimUi.hpp>

namespace rqt_human_radar
{

Radar::Radar()
: rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("Human Radar");
}

Radar::~Radar() {}

void Radar::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new SimUi(0, node_);
  widget_->setMinimumSize(600, 420);
  context.addWidget(widget_);
}

void Radar::shutdownPlugin() {}

void Radar::saveSettings(qt_gui_cpp::Settings &, qt_gui_cpp::Settings &) const
{
}

void Radar::restoreSettings(
  const qt_gui_cpp::Settings &,
  const qt_gui_cpp::Settings &) {}

}  // namespace rqt_human_radar

PLUGINLIB_EXPORT_CLASS(rqt_human_radar::Radar, rqt_gui_cpp::Plugin)
