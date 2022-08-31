/**
 * @file RadarScene.hpp
 * @brief RadarScene class and methods declaration.
 */

#ifndef RQT_ENGAGEMENT_RADAR__RADARSCENE_HPP
#define RQT_ENGAGEMENT_RADAR__RADARSCENE_HPP

#include <QWidget>

#include "rqt_human_radar/RadarCanvas.hpp"

namespace Ui {
class RadarTabs;
};

namespace rqt_human_radar {

class RadarScene :
    public QWidget {
 Q_OBJECT
    public:
        /**
         * @brief Constructor
         */
        RadarScene(QWidget *parent = 0);
        /**
         * @brief Destructor
         */
        virtual ~RadarScene();
    public slots:
        /**
         * @brief function managing the radar canvas.
         * 
         * function called when current tab goes
         * from settings to radar.
         */
        void showRadarCanvas();

    protected:
        /**
         * @brief function managing a resizing event. 
         */
        void resizeEvent(QResizeEvent *event) override;
        /**
         * @brief function managing the window pop-up. 
         */
        void showEvent(QShowEvent *event) override;

    private:
        Ui::RadarTabs *ui_;
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP
