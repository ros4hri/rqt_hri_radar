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
        void attentionConeDegChanged();

    protected:
        // Updates the scribble area where we are painting
        void paintEvent(QPaintEvent *event) override;

    private:
        Ui::RadarCanvas *ui_;

        // Holds the current pen width & color
        int myPenWidth;
        QColor myPenColor;
        QPen fovPen, attentionPen;

        // Stores the image being drawn
        QImage image;

        // Cones amplitude
        double fovAmpl, attentionAmpl;

        // Radar drawing components
        double detectorLength;
        double xOffset, yOffset;
        double fovRectOriginX, fovRectOriginY; 
        double fovX, fovY;
        double attentionX, attentionY;
        double fovStartAngle, fovSpanAngle; 
        double attentionStartAngle, attentionSpanAngle;
};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP