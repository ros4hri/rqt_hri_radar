#ifndef RQT_ENGAGEMENT_RADAR__RADARSCENE_HPP
#define RQT_ENGAGEMENT_RADAR__RADARSCENE_HPP

#include <QTimer>

#include <QDockWidget>
#include <QStringList>
#include <QWidget>

#include <QPen>
#include <QBrush>
#include <QColor>

#include <QImage>
#include <QSize>

#include <QLine>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <hri/hri.h>

#include <geometry_msgs/Vector3Stamped.h>

#include "rqt_engagement_radar/RadarCanvas.hpp"

namespace Ui {
class RadarScene;
};

namespace rqt_engagement_radar {

class RadarScene :
    public QWidget {
 Q_OBJECT
    public:
        RadarScene(QWidget *parent = 0);
        virtual ~RadarScene();

    protected:
        void resizeEvent(QResizeEvent *event) override;
        void showEvent(QShowEvent *event) override;

    private:
        Ui::RadarScene *ui_;
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP