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
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/StringUtils.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "dave_model_systems/UsblTransponder.hh"

// Available interrogation modes
std::vector<std::string> im = {"common", "individual"};

GZ_ADD_PLUGIN(
  dave_model_systems::UsblTransponder, gz::sim::System,
  dave_model_systems::UsblTransponder::ISystemConfigure,
  dave_model_systems::UsblTransponder::ISystemPostUpdate)

namespace dave_model_systems
{

struct UsblTransponder::PrivateData
{
  // Add any private data members here.
  gz::sim::Model model;
  std::string modelName;
  std::string ns;  // 'namespace' is a reserved word in C++, using 'ns' instead.
  std::string m_transceiverDevice;
  std::string m_transceiverID;
  std::string m_transponderDevice;
  std::string m_transponderID;
  double m_noiseSigma;
  double m_noiseMu;
  gz::transport::Node m_gzNode;
  gz::transport::Node::Publisher m_globalPosPub;
  gz::sim::Entity linkEntity;
};

UsblTransponder::UsblTransponder() : dataPtr(std::make_unique<PrivateData>())
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = std::make_shared<rclcpp::Node>("usbl_transponder_node");
  this->log_pub_ = this->ros_node_->create_publisher<std_msgs::msg::String>("/transponder", 10);
}

void UsblTransponder::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
{
  gzdbg << "dave_model_systems::UsblTransponder::Configure on entity: " << _entity << std::endl;

  std_msgs::msg::String msg;
  msg.data = "dave_model_systems::UsblTransponder::Configure on entity: " + std::to_string(_entity);
  this->log_pub_->publish(msg);

  auto model = gz::sim::Model(_entity);
  this->dataPtr->model = model;
  this->dataPtr->modelName = model.Name(_ecm);

  auto links = this->dataPtr->model.Links(_ecm);

  if (!links.empty())
  {
    auto linkEntity = links.front();
    auto linkName = _ecm.Component<gz::sim::components::Name>(linkEntity);
    std::string linkNameStr = linkName->Data();
    gzmsg << "Principal link name: " << linkNameStr << std::endl;
    this->dataPtr->linkEntity = model.LinkByName(_ecm, linkNameStr);
    std::cout << worldPose(this->dataPtr->linkEntity, _ecm) << std::endl;
  }
  else
  {
    gzmsg << "No links found in the model." << std::endl;
  }

  if (!model.Valid(_ecm))
  {
    gzerr << "UsblTransponder plugin should be attached to a model entity. "
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

  // Get transponder device name
  if (!_sdf->HasElement("transponder_device"))
  {
    gzerr << "Missing required parameter <transponder_device>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }
  this->dataPtr->m_transponderDevice = _sdf->Get<std::string>("transponder_device");
  gzmsg << "Transponder device: " << this->dataPtr->m_transponderDevice << std::endl;

  // Get commanding transponders
  if (!_sdf->HasElement("transponder_ID"))
  {
    gzerr << "Missing required parameter <transponder_ID>, "
          << "plugin will not be initialized." << std::endl;
    return;
  }
  this->dataPtr->m_transponderID = _sdf->Get<std::string>("transponder_ID");

  // Get the mean of normal distribution for the noise model
  if (_sdf->HasElement("mu"))
  {
    this->dataPtr->m_noiseMu = _sdf->Get<double>("mu");
  }

  // Get the standard deviation of normal distribution for the noise model
  if (_sdf->HasElement("sigma"))
  {
    this->dataPtr->m_noiseSigma = _sdf->Get<double>("sigma");
  }

  // Define Gazebo publisher for entity's global position
  this->dataPtr->m_globalPosPub = this->dataPtr->m_gzNode.Advertise<gz::msgs::Vector3d>(
    "/" + this->dataPtr->ns + "/" + this->dataPtr->m_transceiverDevice + "_" +
    this->dataPtr->m_transponderID + "/global_position");
}

void UsblTransponder::sendLocation(const gz::sim::EntityComponentManager & _ecm)
{
  // randomly generate from normal distribution for noise
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d(this->dataPtr->m_noiseMu, this->dataPtr->m_noiseSigma);

  gz::math::Pose3d pose = worldPose(this->dataPtr->linkEntity, _ecm);
  gz::math::Vector3<double> position = pose.Pos();
  auto pub_msg = gz::msgs::Vector3d();
  // std::cout << position.X() << " " << position.Y() << " "
  //           << position.Z() << std::endl;
  pub_msg.set_x(position.X() + d(gen));
  pub_msg.set_y(position.Y() + d(gen));
  pub_msg.set_z(position.Z() + d(gen));
  this->dataPtr->m_globalPosPub.Publish(pub_msg);
}

void UsblTransponder::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "dave_model_systems::UsblTransponder::PostUpdate" << std::endl;
    std_msgs::msg::String msg;
    msg.data = "dave_model_systems::UsblTransponder::PostUpdate: namespace = " + this->dataPtr->ns +
               ", model name = " + this->dataPtr->modelName;
    this->log_pub_->publish(msg);
    sendLocation(_ecm);
  }
}

}  // namespace dave_model_systems
