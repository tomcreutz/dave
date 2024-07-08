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

#ifndef DAVE_MODEL_SYSTEMS__USBLTRANSCEIVER_HH_
#define DAVE_MODEL_SYSTEMS__USBLTRANSCEIVER_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace dave_model_systems

{
class UsblTransceiver : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate
{
public:
  UsblTransceiver();
  ~UsblTransceiver() override = default;

  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & eventMgr) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override;

private:
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;

  struct PrivateData;
  std::unique_ptr<PrivateData> dataPtr;
};
}  // namespace dave_model_systems

#endif  // DAVE_MODEL_SYSTEMS__USBLTRANSCEIVER_HH_
