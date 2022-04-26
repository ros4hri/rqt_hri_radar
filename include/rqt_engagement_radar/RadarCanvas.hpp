#ifndef RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP
#define RQT_ENGAGEMENT_RADAR__RADARCANVAS_HPP

#include <QDockWidget>
#include <QStringList>
#include <QWidget>

#include <QColor>

#include <QImage>
#include <QSize>

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

 protected:
  // Updates the scribble area where we are painting
  void paintEvent(QPaintEvent *event) override;

 private:
  Ui::RadarCanvas *ui_;

  // Holds the current pen width & color
  int myPenWidth;
  QColor myPenColor;

  // Stores the image being drawn
  QImage image;

};

} /* namespace */
#endif //RQT_TEMPLATE_PLUGIN_TEMPLATEWIDGET_HPP