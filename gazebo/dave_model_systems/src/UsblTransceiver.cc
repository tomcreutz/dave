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
// System headers
#include <algorithm>
#include <string>
#include <vector>

// External headers (Gazebo, ROS, etc.)
#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "gz/common/StringUtils.hh"
#include "gz/plugin/Register.hh"

// Internal headers (headers from your project)
#include "dave_model_systems/UsblTransceiver.hh"

// available interrogation modes
std::vector<std::string> im = {"common", "individual"};

GZ_ADD_PLUGIN(
  dave_model_systems::UsblTransceiver, gz::sim::System,
  dave_model_systems::UsblTransceiver::ISystemConfigure,
  dave_model_systems::UsblTransceiver::ISystemPostUpdate)

namespace dave_model_systems
{

struct UsblTransceiver::PrivateData
{
  // Add any private data members here.
  gz::sim::Model model;
  std::string modelName;
  std::string ns;  // 'namespace' is a reserved word in C++, using 'ns' instead.
  std::string m_transceiverDevice;
  std::string m_transceiverID;
  std::string m_transponderDevice;
  std::vector<std::string> m_deployedTransponders;
  bool m_enablePingerScheduler;
  std::string m_transponderAttachedObject;
  std::string m_interrogationMode;
  gz::transport::Node node;
};

UsblTransceiver::UsblTransceiver() : dataPtr(std::make_unique<PrivateData>())
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = std::make_shared<rclcpp::Node>("usbl_transceiver_node");
  this->log_pub_ = this->ros_node_->create_publisher<std_msgs::msg::String>("/gazebo_logs", 10);
}

void UsblTransceiver::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
{
  gzdbg << "dave_model_systems::UsblTransceiver::Configure on entity: " << _entity << std::endl;

  std_msgs::msg::String msg;
  msg.data = "dave_model_systems::UsblTransceiver::Configure on entity: " + std::to_string(_entity);
  this->log_pub_->publish(msg);

  auto model = gz::sim::Model(_entity);
  this->dataPtr->model = model;
  this->dataPtr->modelName = model.Name(_ecm);

  if (!model.Valid(_ecm))
  {
    gzerr << "UsblTransceiver plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  // Grab namespace from SDF
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }
  this->dataPtr->ns = _sdf->Get<std::string>("namespace");

  // Obtain transceiver device name from SDF
  if (!_sdf->HasElement("transceiver_device"))
  {
    gzerr << "Missing required parameter <transceiver_device>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }

  this->dataPtr->m_transceiverDevice = _sdf->Get<std::string>("transceiver_device");
  gzmsg << "Entity: " << this->dataPtr->m_transceiverDevice << std::endl;

  // Get transceiver device id
  if (!_sdf->HasElement("transceiver_ID"))
  {
    gzerr << "Missing required parameter <transceiver_ID>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }

  this->dataPtr->m_transceiverID = _sdf->Get<std::string>("transceiver_ID");

  // get transponder device name
  if (!_sdf->HasElement("transponder_device"))
  {
    gzerr << "Missing required parameter <transponder_device>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }
  this->dataPtr->m_transponderDevice = _sdf->Get<std::string>("transponder_device");
  gzmsg << "Transponder device: " << this->dataPtr->m_transponderDevice << std::endl;

  // get commanding transponders
  if (!_sdf->HasElement("transponder_ID"))
  {
    gzerr << "Missing required parameter <transponder_ID>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }

  auto transponders = gz::common::Split(_sdf->Get<std::string>("transponder_ID"), ',');
  gzmsg << "Current deployed transponders are: \n";

  for (auto & transponder : transponders)
  {
    gzmsg << transponder << std::endl;
    this->dataPtr->m_deployedTransponders.push_back(transponder);
  }

  // Enable automation of sending pings to transponder
  if (!_sdf->HasElement("enable_ping_scheduler"))
  {
    gzerr << "Missing required parameter <enable_ping_scheduler>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }

  this->dataPtr->m_enablePingerScheduler = _sdf->Get<bool>("enable_ping_scheduler");
  gzmsg << "pinger enable? " << this->dataPtr->m_enablePingerScheduler << std::endl;

  // Get object that transponder attached to
  if (!_sdf->HasElement("transponder_attached_object"))
  {
    gzerr << "Missing required parameter <transponder_attached_object>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }

  this->dataPtr->m_transponderAttachedObject =
    _sdf->Get<std::string>("transponder_attached_object");
  gzmsg << "Transponder attached object: " << this->dataPtr->m_transponderAttachedObject
        << std::endl;

  /*  interrogation mode - 2 options
   *  II (individual interrogation) <----->  CRS (common response signal)
   *  CI (common interrogation)     <----->  IRS (individual response
   *                                         signal) from transponder_01
   *                                    ͱ->  IRS from transponder_02
   *                                    ͱ->  IRS from transponder_03
   *                                            ⋮
   */

  if (_sdf->HasElement("interrogation_mode"))
  {
    std::string interrogation_mode = _sdf->Get<std::string>("interrogation_mode");
    if (std::find(im.begin(), im.end(), interrogation_mode) != im.end())
    {
      gzmsg << interrogation_mode << " interrogation mode is used" << std::endl;
      this->dataPtr->m_interrogationMode = interrogation_mode;
    }
    else
    {
      gzmsg << "Specified interrogation mode is unavailable, "
            << "Common mode is used" << std::endl;
      this->dataPtr->m_interrogationMode = "common";
    }
  }
  else
  {
    gzmsg << "Interrogation mode is not specified, Common mode is used" << std::endl;
    this->dataPtr->m_interrogationMode = "common";
  }
}

void UsblTransceiver::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "dave_model_systems::UsblTransceiver::PostUpdate" << std::endl;
    std_msgs::msg::String msg;
    msg.data = "dave_model_systems::UsblTransceiver::PostUpdate: namespace = " + this->dataPtr->ns +
               ", model name = " + this->dataPtr->modelName;
    this->log_pub_->publish(msg);
  }
}

}  // namespace dave_model_systems
