#ifndef RQT_ENGAGEMENT_RADAR__RADAR_HPP
#define RQT_ENGAGEMENT_RADAR__RADAR_HPP

#include <QStringList>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <unistd.h>

namespace rqt_engagement_radar {

class Radar;

class Radar : public rqt_gui_cpp::Plugin {
 Q_OBJECT
 public:
  Radar();
  virtual ~Radar();

  virtual void initPlugin(qt_gui_cpp::PluginContext &context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() ch ficonst;
  //void triggerConfiguration();

 private:
  QWidget* widget_;
};

} /* namespace */

#endif /* RQT_ENGAGEMENT_RADAR__RADAR_HPP */