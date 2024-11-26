// Copyright 2024 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rqt_human_radar/SimScene.hpp"

#include "rqt_human_radar/RemotePersonItem.hpp"

using namespace std::chrono_literals;

namespace rqt_human_radar
{

RemotePersonItem::RemotePersonItem(
  rclcpp::Node::SharedPtr node,
  hri::ConstPersonPtr person,
  const std::string & resource_path,
  const std::string & reference_frame_id,
  bool is_static)
: PersonItem(node, person->id(), "oro:Human",
    resource_path + "/res/adult.svg"),
  frame_id_(person->frame()), node_(node),
  person_(person), reference_frame_id_(reference_frame_id)
{
  setPhysicalWidth(.3);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a timer to periodically broadcast the transform
  if (!is_static) {
    timer_ = node_->create_wall_timer(
      50ms, std::bind(&RemotePersonItem::updatePosition, this));
  }
}

RemotePersonItem::~RemotePersonItem()
{
  if (timer_) {
    timer_->cancel();
  }
}
void RemotePersonItem::updatePosition()
{
  try {
    auto transformStamped = tf_buffer_->lookupTransform(
      reference_frame_id_, frame_id_, tf2::TimePointZero);

    QPointF newPosition(transformStamped.transform.translation.x *
      SimScene::pixelsPerMeter,
      transformStamped.transform.translation.y *
      -SimScene::pixelsPerMeter);  // Invert Y-axis

    // Extract rotation as a 2D angle
    const auto & rotation = transformStamped.transform.rotation;
    tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);

    // Convert quaternion to a yaw angle (rotation about Z-axis)
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // we can not update the position directly from the ROS callback thread,
    // so we use invokeMethod to do it in the main thread
    QMetaObject::invokeMethod(
      this, [this, newPosition, yaw]() {
        setPos(newPosition);
        setRotation(-yaw * 180.0 / M_PI);
        scene()->update();  // required to trigger a repaint of background
      });
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform: %s", ex.what());
  }
}

}  // namespace rqt_human_radar
