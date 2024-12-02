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

#include <random>

#include "rqt_human_radar/SemanticObject.hpp"

namespace rqt_human_radar
{

std::string SemanticObject::generateRandomSuffix(size_t length)
{
  static const char charset[] = "abcdefghijklmnopqrstuvwxyz";
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<size_t> dist(0, sizeof(charset) - 2);

  std::string result;
  result.reserve(length);
  for (size_t i = 0; i < length; ++i) {
    result += charset[dist(rng)];
  }
  return result;
}

SemanticObject::SemanticObject(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & classname, bool randomize_id)
: name_(name), classname_(classname), node_(node)
{
  kb_add_pub_ =
    node_->create_publisher<std_msgs::msg::String>("/kb/add_fact", 10);
  kb_remove_pub_ =
    node_->create_publisher<std_msgs::msg::String>("/kb/remove_fact", 10);

  if (randomize_id) {
    id_ = name_ + "_" + generateRandomSuffix();
    // convert the id to lowercase
    std::transform(
      id_.begin(), id_.end(), id_.begin(),
      [](unsigned char c) {return std::tolower(c);});
  } else {
    id_ = name;
  }

  // create a connection to the ROS2 topic /kb/add and publish the classname
  Triple triple = std::make_tuple(id_, "rdf:type", classname_);
  static_triples_.push_back(triple);
  kb_add_pub_->publish(toMsg(triple));
}

SemanticObject::~SemanticObject()
{
  for (const auto & triple : static_triples_) {
    kb_remove_pub_->publish(toMsg(triple));
  }
}

void SemanticObject::addProperty(
  const std::string & property,
  const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = id_ + " " + property + " " + value;

  kb_add_pub_->publish(msg);
}

void SemanticObject::removeProperty(
  const std::string & property,
  const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = id_ + " " + property + " " + value;

  kb_remove_pub_->publish(msg);
}

std_msgs::msg::String SemanticObject::toMsg(const Triple & triple)
{
  std_msgs::msg::String msg;
  msg.data = std::get<0>(triple) + " " + std::get<1>(triple) + " " + std::get<2>(triple);
  return msg;
}

/** add & publish triples that are not yet in static_triples_; remove triples that are in static_triples_ but not in triples */
void SemanticObject::updateStaticTriples(const std::vector<Triple> & triples)
{

  decltype(static_triples_) new_static_triples_;

  for (const auto & triple : triples) {
    if (std::find(
        static_triples_.begin(), static_triples_.end(),
        triple) == static_triples_.end())
    {

      kb_add_pub_->publish(toMsg(triple));

      new_static_triples_.push_back(triple);
    }
  }

  for (const auto & triple : static_triples_) {
    if (std::find(triples.begin(), triples.end(), triple) == triples.end()) {
      kb_remove_pub_->publish(toMsg(triple));
      static_triples_.erase(
        std::remove(static_triples_.begin(), static_triples_.end(), triple),
        static_triples_.end());
    }
  }

  // add the new triples to the static triples
  static_triples_.insert(
    static_triples_.end(), new_static_triples_.begin(), new_static_triples_.end());


}
}  // namespace rqt_human_radar
