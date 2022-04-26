#include "rqt_engagement_radar/RadarCanvas.hpp"

#include "ui_radar_canvas.h"

#include <QPainter>

namespace rqt_engagement_radar {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

RadarCanvas::RadarCanvas(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::RadarCanvas()) {
  ui_->setupUi(this);
  
  image = QImage(QSize(600, 600), QImage::Format_RGB32);
  image.fill(qRgb(255, 255, 255));
  
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), image);

  update();
}

RadarCanvas::~RadarCanvas() {
  //confirmClose();

  delete ui_;
}

void RadarCanvas::paintEvent(QPaintEvent *event){
  QPainter painter(this);

  // Draws the rectangle where the image needs to
  // be updated
  painter.drawImage(QPoint(0, 0), image);
}

} /* namespace */