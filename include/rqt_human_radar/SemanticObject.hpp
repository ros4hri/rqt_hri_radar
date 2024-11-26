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

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace rqt_human_radar
{
/** @brief A semantic object in the simulation
 *
 *  It holds a unique identifier, an ROS2 node handler,
 *  and offer methods to add/remove semantic properties
 *  in the knowledge base via the /kb/add_fact and /kb/remove_fact
 *  topics.
 **/
class SemanticObject
{
public:
  SemanticObject(
    rclcpp::Node::SharedPtr node, const std::string & name,
    const std::string & classname = "owl:Thing",
    bool randomize_id = false);

  ~SemanticObject();

  void addProperty(const std::string & property, const std::string & value);
  void removeProperty(const std::string & property, const std::string & value);

  static std::string generateRandomSuffix(size_t length = 5);

  std::string getId() const {return id_;}
  std::string getName() const {return name_;}
  std::string getClassname() const {return classname_;}

protected:
  std::string name_;
  std::string classname_;
  std::string id_;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kb_add_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kb_remove_pub_;
};

}  // namespace rqt_human_radar
