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
 */

#include <gz/msgs/vector3d.pb.h>
#include <algorithm>
#include <iostream>

#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include "gz/common/StringUtils.hh"
#include "gz/plugin/Register.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/World.hh"

#include "dave_ros_gz_plugins/SphericalCoords.hh"

GZ_ADD_PLUGIN(
  dave_ros_gz_plugins::SphericalCoords, gz::sim::System,
  dave_ros_gz_plugins::SphericalCoords::ISystemConfigure,
  dave_ros_gz_plugins::SphericalCoords::ISystemPostUpdate)

namespace dave_ros_gz_plugins
{

struct SphericalCoords::PrivateData
{
  // Add any private data members here.
  gz::sim::Model model;
  std::string modelName;
  std::string ns;
  gz::sim::EntityComponentManager * ecm;
};

SphericalCoords::SphericalCoords() : dataPtr(std::make_unique<PrivateData>()) {}

void SphericalCoords::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
{
  gzdbg << "dave_ros_gz_plugins::UsblTransceiver::Configure on entity: " << _entity << std::endl;

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = std::make_shared<rclcpp::Node>("usbl_transceiver_node");

  auto model = gz::sim::Model(_entity);
  this->dataPtr->model = model;
  this->dataPtr->modelName = model.Name(_ecm);
  this->dataPtr->ecm = &_ecm;
}

void SphericalCoords::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
}

}  // namespace dave_ros_gz_plugins
