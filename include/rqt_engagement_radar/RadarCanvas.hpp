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
class RadarCanvas;
};

namespace rqt_engagement_radar {

class RadarCanvas :
    public QWidget {
 Q_OBJECT
    public:
        RadarCanvas(QWidget *parent = 0);
        virtual ~RadarCanvas();
 
    public slots:
        void fovConeDegChanged();
        void fovConeRangeChanged();
        void attentionConeDegChanged();
        void attentionConeRangeChanged();

    protected:
        void paintEvent(QPaintEvent *event) override;
        void resizeEvent(QResizeEvent *event) override;

    private:
        Ui::RadarCanvas *ui_;

        QTimer *timer_;

        hri::HRIListener hriListener_;
        tf::TransformListener tfListener_;
        geometry_msgs::Vector3Stamped versor_;

        // Drawing and painting objects
        QPen fovPen, attentionPen, rangePen;
        QBrush fovBrush, attentionBrush;

        // Stores the image being drawn
        QImage background; // TODO: evaluating different styles rather than just fully white background
        QImage robotImage, personImage;
        bool robotImageFound, personImageFound;
        std::string package, robotImageFile, personImageFile;
        std::map<std::string, std::vector<double>> peoplePosition;

        // Cones amplitude
        double fovAmpl, attentionAmpl;

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
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP