#include <iostream>

#include "rqt_engagement_radar/RadarCanvas.hpp"

#include "ui_radar_canvas.h"

#include <QPainter>
#include <QRectF>
#include <cmath>


namespace rqt_engagement_radar {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

RadarCanvas::RadarCanvas(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::RadarCanvas()) {
  ui_->setupUi(this);
  
  image = QImage(QSize(800, 600), QImage::Format_RGB32);
  image.fill(qRgb(255, 255, 255));
  
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), image);

  connect(ui_->fovConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::fovConeDegChanged);
  connect(ui_->attentionConeDeg, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RadarCanvas::attentionConeDegChanged);

  detectorLength = 400;
  xOffset = 50; // Random value
  yOffset = 300; // Image length (in this case 600) / 2 ==> 3000

  fovPen = QPen(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  attentionPen = QPen(Qt::red, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

  update();
}

RadarCanvas::~RadarCanvas() {
  //confirmClose();

  delete ui_;
}

void RadarCanvas::fovConeDegChanged(){
  fovAmpl = ui_->fovConeDeg->value(); 
  update();
}

void RadarCanvas::attentionConeDegChanged(){
  attentionAmpl = ui_->attentionConeDeg->value();
  update();
}

void RadarCanvas::paintEvent(QPaintEvent *event){
  QPainter painter(this);
  painter.setPen(fovPen);

  image = QImage(QSize(800, 600), QImage::Format_RGB32);
  image.fill(qRgb(255, 255, 255));

  painter.drawImage(QPoint(0, 0), image);

  fovRectOriginX = xOffset - detectorLength;
  fovRectOriginY = yOffset - detectorLength;
  fovStartAngle = -(floor(fovAmpl/2)*16 + ((fovAmpl-floor(fovAmpl/2))/100*16));
  fovSpanAngle = floor(fovAmpl)*16 + ((fovAmpl-floor(fovAmpl))/100*16); 

  QRectF fovRectangle(QPoint(fovRectOriginX, fovRectOriginY), QSize(detectorLength*2, detectorLength*2));
  painter.drawPie(fovRectangle, fovStartAngle, fovSpanAngle);

  painter.setPen(attentionPen);
  attentionStartAngle = -(floor(attentionAmpl/2)*16 + ((attentionAmpl-floor(attentionAmpl/2))/100*16));
  attentionSpanAngle = floor(attentionAmpl)*16 + ((attentionAmpl-floor(attentionAmpl))/100*16);

  painter.drawPie(fovRectangle, attentionStartAngle, attentionSpanAngle);

  //std::cout<<fovRectOriginX<<"\t"<<fovRectOriginY<<"\t"<<sin(fovAmpl/360*M_PI)<<std::endl;
  //std::cout<<fovStartAngle<<"\t"<<fovSpanAngle<<"\t"<<std::endl;
}

} /* namespace */