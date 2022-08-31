/**
 * @file RadarCanvas.hpp
 * @brief Defines the class for the RadarCanvas widget.
 * 
 * The widget where the radar background
 * and icons for the rqt-humans-radar are painted
 * taking into account the preferences expressed
 * by the user in the settings tab.
 */
#ifndef RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP
#define RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP

#include <QTimer>

#include <QDockWidget>
#include <QStringList>
#include <QWidget>
#include <QtSvg/QSvgWidget>
#include <QtSvg/QSvgRenderer>

#include <QPen>
#include <QBrush>
#include <QColor>
#include <QFont>

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

namespace rqt_human_radar {

class RadarCanvas :
    public QWidget {
 Q_OBJECT
    public:
        /** 
         * @brief Constructor
         */ 
        RadarCanvas(QWidget *parent, Ui::RadarTabs* ui_);
        /** 
         * @brief Destructor
         */
        virtual ~RadarCanvas();
    public slots:
        /** 
         * @brief Updating the pixel-per-meter value.
         */
        void updatePixelPerMeter();
        /** 
         * @brief Reading the user preference about showing or not people ID.
         * 
         * The preference is expressed in settings,
         * through a tick-box. Currently, person ID = <body_id>
         */
        void showId();

    protected:
        /** 
         * @brief overriding the paintEvent virtual function. 
         * 
         * Inherited from QWidget. Paints the entire radar canvas.
         */
        void paintEvent(QPaintEvent *event) override;
        /** 
         * @brief overriding the resizeEvent virtual function. 
         * 
         * Inherited from QWidget. Updates some painting parameters 
         * according to the new size of the window.
         */
        void resizeEvent(QResizeEvent *event) override;
        /**
         * @brief overriding the mousePressEvent virtual function
         * 
         * Inherited from QWidget. Stores the id of the person 
         * icon the user has clicked on, if any. 
         */
        void mousePressEvent(QMouseEvent* event) override;

    private:
        /**
         * @brief returns whether or not a point is inside the canvas.
         */
        bool inScreen(double& x, double& y) const;
        /**
         * @brief updates the number of arcs to draw.
         * 
         * The numbers of arcs to draw is defined by the current
         * size of the canvas. 
         */
        void updateArcsToDraw();

        QTimer *timer_;

        hri::HRIListener hriListener_;
        tf::TransformListener tfListener_;
        geometry_msgs::Vector3Stamped versor_;

        // Drawing and painting objects
        QPen rangePen_;
        QBrush oddBrush_, evenBrush_;
        QFont font_, anglesFont_;

        // Stores the image being drawn
        QImage robotImage_;
        bool robotImageFound, personImageFound;
        std::string package_, robotImageFile_, personSvgFile_;
        std::map<std::string, QPolygon> peoplePosition_;

        // Svg renderer
        QSvgRenderer svgRenderer_;
        bool svgRendererInitialized_;

        int pixelPerMeter_;

        int arcsToDraw_;

        Qt::CheckState showIdValue_;

        // Radar drawing components
        double xOffset_, yOffset_;

        // New stuff to avoid using ui
        QWidget* widget_;
        Ui::RadarTabs* ui_;

        // ID clicked with mouse
        std::string idClicked_;

        // Reference frame
        std::string referenceFrame_;

};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP