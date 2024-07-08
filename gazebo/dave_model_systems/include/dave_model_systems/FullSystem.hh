/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef DAVE_MODEL_SYSTEMS__FULLSYSTEM_HH_
#define DAVE_MODEL_SYSTEMS__FULLSYSTEM_HH_

#include <memory>

#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace dave_model_systems
{
class FullSystem : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate,
                   public gz::sim::ISystemUpdate,
                   public gz::sim::ISystemPostUpdate,
                   public gz::sim::ISystemReset
{
public:
  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _element,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager) override;

public:
  void PreUpdate(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

public:
  void Update(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

public:
  void PostUpdate(
    const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm) override;

public:
  void Reset(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
};
}  // namespace dave_model_systems
#endif  // DAVE_MODEL_SYSTEMS__FULLSYSTEM_HH_
