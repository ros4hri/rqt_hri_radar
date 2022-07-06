#ifndef RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP
#define RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP

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

namespace Ui {
class RadarTabs;
};   

namespace rqt_engagement_radar {

class RadarCanvas :
    public QWidget {
 Q_OBJECT
    public:
        RadarCanvas(QWidget *parent, Ui::RadarTabs* ui_);
        virtual ~RadarCanvas();
    public slots:
        void fovConeDegChanged();
        void fovConeRangeChanged();
        void attentionConeDegChanged();
        void attentionConeRangeChanged();

    protected:
        void paintEvent(QPaintEvent *event) override;
        void resizeEvent(QResizeEvent *event) override;
        void showEvent(QShowEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;

    private:
        QTimer *timer_;

        hri::HRIListener hriListener_;
        tf::TransformListener tfListener_;
        geometry_msgs::Vector3Stamped versor_;

        // Drawing and painting objects
        QPen fovPen, attentionPen, rangePen;
        QBrush fovBrush, attentionBrush;
        QBrush oddBrush, evenBrush;

        // Stores the image being drawn
        QImage background; // TODO: evaluating different styles rather than just fully white background
        QImage robotImage, personImage;
        bool robotImageFound, personImageFound;
        std::string package, robotImageFile, personImageFile;
        std::map<std::string, QPolygon> peoplePosition;

        // Cones amplitude
        double fovAmpl, attentionAmpl;
        int pixelPerMeter;

        int arcsToDraw;

        // Radar drawing components
        double detectorLength;
        double xOffset, yOffset;
        double fovRectOriginX, fovRectOriginY; 
        double fovX, fovY;
        double attentionRectOriginX, attentionRectOriginY;
        double attentionX, attentionY;
        double fovStartAngle, fovSpanAngle; 
        double attentionStartAngle, attentionSpanAngle;
        double fovRange, attentionRange;

        // New stuff to avoid using ui
        QWidget* widget_;
        Ui::RadarTabs* ui_;

        // ID hovered by the mouse
        std::string idHovered;
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP