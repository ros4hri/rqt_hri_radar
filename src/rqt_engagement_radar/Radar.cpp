#include "rqt_engagement_radar/Radar.hpp"

#include <pluginlib/class_list_macros.h>
#include <QGridLayout>
#include <QStringList>

namespace rqt_engagement_radar {

Radar::Radar()
    : rqt_gui_cpp::Plugin(){
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("Radar");
}

Radar::~Radar() {
}

void Radar::initPlugin(qt_gui_cpp::PluginContext &context) {
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

PLUGINLIB_EXPORT_CLASS(rqt_engagement_radar::Radar, rqt_gui_cpp::Plugin)