/**
 * @file Radar.cpp
 * @brief Definition of the methods declared in Radar.hpp
 */

#include "rqt_human_radar/Radar.hpp"

#include <pluginlib/class_list_macros.h>

#include <rqt_human_radar/RadarCanvas.hpp>
#include <rqt_human_radar/RadarScene.hpp>

namespace rqt_human_radar {

Radar::Radar()
    : rqt_gui_cpp::Plugin(),
      widget_(0){
  setObjectName("Radar");
}

Radar::~Radar() {
}

void Radar::initPlugin(qt_gui_cpp::PluginContext &context) {
  widget_ = new RadarScene();
  widget_->setMinimumSize(600, 420);
  context.addWidget(widget_);
}

void Radar::shutdownPlugin() {
}

void Radar::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                  qt_gui_cpp::Settings &instance_settings) const {
}

void Radar::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                     const qt_gui_cpp::Settings &instance_settings) {
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(rqt_human_radar::Radar, rqt_gui_cpp::Plugin)
