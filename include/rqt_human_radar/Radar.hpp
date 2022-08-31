/**
 * @file Radar.hpp
 * @brief Defines the class for the Radar widget.
 */

#ifndef RQT_ENGAGEMENT_RADAR__RADAR_HPP
#define RQT_ENGAGEMENT_RADAR__RADAR_HPP

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <unistd.h>

namespace rqt_human_radar {

class Radar;

class Radar : public rqt_gui_cpp::Plugin {
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
    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    /** 
     * @brief template function for rqt plugins
     */
    virtual void shutdownPlugin();
    /** 
     * @brief template function for rqt plugins
     */
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
    /** 
     * @brief template function for rqt plugins
     */
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);
    private:
    QWidget* widget_;
};

} /* namespace */

#endif /* RQT_ENGAGEMENT_RADAR__RADAR_HPP */