#pragma once

#include <QSvgRenderer>
#include <QDomDocument>
#include <string>
#include <QGraphicsScene>

#include <rclcpp/rclcpp.hpp>

namespace rqt_human_radar
{

class EnvironmentLoader
{
public:
  EnvironmentLoader() {}

  void loadMap(rclcpp::Node::SharedPtr node, QGraphicsScene * scene, const std::string & filename);

private:
  QRectF getElementBounds(
    rclcpp::Node::SharedPtr node, const std::string & elementId,
    const std::string & label);

  QSvgRenderer renderer_;
  QDomDocument doc_;


};
}   // namespace rqt_human_radar
