#ifndef RQT_ENGAGEMENT_RADAR__RADARSCENE_HPP
#define RQT_ENGAGEMENT_RADAR__RADARSCENE_HPP

#include <QWidget>

#include "rqt_engagement_radar/RadarCanvas.hpp"

namespace Ui {
class RadarTabs;
};

namespace rqt_engagement_radar {

class RadarScene :
    public QWidget {
 Q_OBJECT
    public:
        RadarScene(QWidget *parent = 0);
        virtual ~RadarScene();
    public slots:
        void showRadarCanvas();

    protected:
        void resizeEvent(QResizeEvent *event) override;
        void showEvent(QShowEvent *event) override;

    private:
        Ui::RadarTabs *ui_;
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP