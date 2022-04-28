#ifndef RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP
#define RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP

#include <QDockWidget>
#include <QStringList>
#include <QWidget>

#include <QPen>
#include <QColor>

#include <QImage>
#include <QSize>

#include <QLine>

#include <ros/ros.h>

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

        ros::NodeHandle nh;

        // Holds the current pen width & color
        QPen fovPen, attentionPen, rangePen;

        // Stores the image being drawn
        QImage background; // TODO: evaluating different styles rather than just fully white background
        QImage robotImage, personImage;
        bool robotImageFound, personImageFound;
        std::string package, robotImageFile, personImageFile;
        std::map<std::string, std::vector<double>> peoplePosition;

        // Cones amplitude
        double fovAmpl, attentionAmpl;

        // Radar drawing components
        double detectorLength;
        double xOffset, yOffset;
        double fovRectOriginX, fovRectOriginY; 
        double fovX, fovY;
        double attentionRectOriginX, attentionRectOriginY;
        double attentionX, attentionY;
        double fovStartAngle, fovSpanAngle; 
        double attentionStartAngle, attentionSpanAngle;
        double fovRange, attentionRange, circleRange;
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP