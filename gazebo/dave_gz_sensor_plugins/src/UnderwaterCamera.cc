/*
 * Copyright (C) 2024 Rakesh Vivekanandan
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

#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include "gz/plugin/Register.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/DepthCamera.hh"
#include <sdf/Camera.hh>
#include "dave_gz_sensor_plugins/UnderwaterCamera.hh"

GZ_ADD_PLUGIN(
  dave_gz_sensor_plugins::UnderwaterCamera, gz::sim::System,
  dave_gz_sensor_plugins::UnderwaterCamera::ISystemConfigure,
  dave_gz_sensor_plugins::UnderwaterCamera::ISystemPostUpdate)

namespace dave_gz_sensor_plugins
{

struct UnderwaterCamera::PrivateData
{
  // Add any private data members here.
  std::mutex mutex_;
  gz::transport::Node gz_node;
  std::string image_topic;
  std::string depth_image_topic;
  std::string camera_info_topic;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

  /// \brief Width of the image.
  unsigned int width;

  /// \brief Height of the image.
  unsigned int height;

  /// \brief Camera intrinsics.
  double fx;
  double fy;
  double cx;
  double cy;

  /// \brief Temporarily store pointer to previous depth image.
  const float * lastDepth;

  /// \brief Latest simulated image.
  unsigned char * lastImage;

  /// \brief Depth to range lookup table (LUT)
  float* depth2rangeLUT;

  /// \brief Attenuation constants per channel (RGB)
  float attenuation[3];

  /// \brief Background constants per channel (RGB)
  unsigned char background[3];
};

UnderwaterCamera::UnderwaterCamera() : dataPtr(std::make_unique<PrivateData>()) {}

UnderwaterCamera::~UnderwaterCamera() {
  if (this->dataPtr->lastImage)
  {
    delete[] this->dataPtr->lastImage;
  }

  if (this->dataPtr->depth2rangeLUT)
  {
    delete[] this->dataPtr->depth2rangeLUT;
  }
}

void UnderwaterCamera::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
{
  gzdbg << "dave_gz_sensor_plugins::UnderwaterCamera::Configure on entity: " << _entity << std::endl;

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = std::make_shared<rclcpp::Node>("underwater_camera_node");

  auto sensorComp = _ecm.Component<gz::sim::components::DepthCamera>(_entity);
  if (!sensorComp)
  {
    gzerr << "UnderwaterCamera plugin requires a DepthCamera component" << std::endl;
    return;
  }

  sdf::Sensor sensorSdf = sensorComp->Data();
  if(sensorSdf.Type() != sdf::SensorType::DEPTH_CAMERA)
  {
    gzerr << "Sensor type is not depth camera" << std::endl;
    return;
  }

  if(sensorSdf.CameraSensor() == nullptr)
  {
    gzerr << "Camera sensor is null" << std::endl;
    return;
  }
  
  sdf::Camera *cameraSdf = sensorSdf.CameraSensor();
  this->dataPtr->width = cameraSdf->ImageWidth();
  this->dataPtr->height = cameraSdf->ImageHeight();
  this->dataPtr->fx = cameraSdf->LensIntrinsicsFx();
  this->dataPtr->fy = cameraSdf->LensIntrinsicsFy();
  this->dataPtr->cx = cameraSdf->LensIntrinsicsCx();
  this->dataPtr->cy = cameraSdf->LensIntrinsicsCy();

  // Grab camera topic from SDF
  if (!_sdf->HasElement("camera_topic"))
  {
    this->dataPtr->image_topic = "";
    gzmsg << "Camera topic set to default:  " << this->dataPtr->image_topic << std::endl;
  }
  else
  {
    this->dataPtr->image_topic = _sdf->Get<std::string>("camera_topic");
    gzmsg << "Camera topic: " << this->dataPtr->image_topic << std::endl;
  }

  // Grab depth image topic from SDF
  if (!_sdf->HasElement("depth_topic"))
  {
    this->dataPtr->depth_image_topic = "";
    gzmsg << "Depth topic set to default:  " << this->dataPtr->depth_image_topic << std::endl;
  }
  else
  {
    this->dataPtr->image_topic = _sdf->Get<std::string>("depth_topic");
    gzmsg << "Depth topic: " << this->dataPtr->image_topic << std::endl;
  }

  if(!_sdf->HasElement("attenuationR"))
  {
    this->dataPtr->attenuation[0] = 1.f / 30.f;
  }
  else
  {
    this->dataPtr->attenuation[0] = _sdf->Get<float>("attenuationR");
  }

  if(!_sdf->HasElement("attenuationG"))
  {
    this->dataPtr->attenuation[1] = 1.f / 30.f;
  }
  else
  {
    this->dataPtr->attenuation[1] = _sdf->Get<float>("attenuationG");
  }

  if(!_sdf->HasElement("attenuationB"))
  {
    this->dataPtr->attenuation[2] = 1.f / 30.f;
  }
  else
  {
    this->dataPtr->attenuation[2] = _sdf->Get<float>("attenuationB");
  }

  if(!_sdf->HasElement("backgroundR"))
  {
    this->dataPtr->background[0] = (unsigned char)0;
  }
  else
  {
    this->dataPtr->background[0] = (unsigned char)_sdf->Get<int>("backgroundR");
  }

  if(!_sdf->HasElement("backgroundG"))
  {
    this->dataPtr->background[1] = (unsigned char)0;
  }
  else
  {
    this->dataPtr->background[1] = (unsigned char)_sdf->Get<int>("backgroundG");
  }

  if(!_sdf->HasElement("backgroundB"))
  {
    this->dataPtr->background[2] = (unsigned char)0;
  }
  else
  {
    this->dataPtr->background[2] = (unsigned char)_sdf->Get<int>("backgroundB");
  }


  // Create and fill depth2range LUT
  this->dataPtr->depth2rangeLUT = new float[this->dataPtr->width * this->dataPtr->height];
  float * lutPtr = this->dataPtr->depth2rangeLUT;
  for (int v = 0; v < this->dataPtr->height; v++)
  {
      double y_z = (v - this->dataPtr->cy)/this->dataPtr->fy;
      for (int u = 0; u < this->dataPtr->width; u++)
      {
          double x_z = (u - this->dataPtr->cx)/this->dataPtr->fx;
          // Precompute the per-pixel factor in the following formula:
          // range = || (x, y, z) ||_2
          // range = || z * (x/z, y/z, 1.0) ||_2
          // range = z * || (x/z, y/z, 1.0) ||_2
          *(lutPtr++) = sqrt(1.0 + x_z*x_z + y_z*y_z);
      }
  }

  // Gazebo camera subscriber
  std::function<void(const gz::msgs::Image &)> camera_callback =
    std::bind(&UnderwaterCamera::CameraCallback, this, std::placeholders::_1);

  this->dataPtr->gz_node.Subscribe(this->dataPtr->image_topic, camera_callback);

  // Gazebo depth image subscriber
  std::function<void(const gz::msgs::Image &)> depth_callback =
    std::bind(&UnderwaterCamera::DepthImageCallback, this, std::placeholders::_1);

  this->dataPtr->gz_node.Subscribe(this->dataPtr->depth_image_topic, depth_callback);

  // ROS2 publisher
  this->dataPtr->image_pub =
    this->ros_node_->create_publisher<sensor_msgs::msg::Image>(this->dataPtr->image_topic, 1);
}

void UnderwaterCamera::CameraCallback(const gz::msgs::Image & msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);

  gzmsg << "dave_gz_sensor_plugins::UnderwaterCamera::CameraCallback" << std::endl;

  // Convert Gazebo image to OpenCV image
  const cv::Mat image(
    msg.height(), msg.width(), CV_8UC3, const_cast<unsigned char *>(reinterpret_cast<const unsigned char *>(msg.data().c_str())));

  // Convert depth image to OpenCV image
  const cv::Mat depth(
    msg.height(), msg.width(), CV_32FC1, const_cast<float *>(this->dataPtr->lastDepth));

  // Create output image
  cv::Mat output(msg.height(), msg.width(), CV_8UC3, this->dataPtr->lastImage);

  // Simulate underwater
  this->SimulateUnderwater(image, depth, output);

  // Publish simulated image
  sensor_msgs::msg::Image ros_image;
  ros_image.header.stamp = this->ros_node_->now();
  ros_image.height = msg.height();
  ros_image.width = msg.width();
  ros_image.encoding = "bgr8";
  ros_image.is_bigendian = false;
  ros_image.step = msg.width() * 3;
  ros_image.data = std::vector<unsigned char>(output.data, output.data + output.total() * output.elemSize());

  this->dataPtr->image_pub->publish(ros_image);

}

void UnderwaterCamera::DepthImageCallback(const gz::msgs::Image & msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);

  gzmsg << "dave_gz_sensor_plugins::UnderwaterCamera::DepthImageCallback" << std::endl;

  this->dataPtr->lastDepth = reinterpret_cast<const float *>(msg.data().c_str());
}

void UnderwaterCamera::SimulateUnderwater(const cv::Mat& _inputImage,
  const cv::Mat& _inputDepth, cv::Mat& _outputImage)
{
  const float * lutPtr = this->dataPtr->depth2rangeLUT;
  for (unsigned int row = 0; row < this->dataPtr->height; row++)
  {
    const cv::Vec3b* inrow = _inputImage.ptr<cv::Vec3b>(row);
    const float* depthrow = _inputDepth.ptr<float>(row);
    cv::Vec3b* outrow = _outputImage.ptr<cv::Vec3b>(row);

    for (int col = 0; col < this->dataPtr->width; col++)
    {
      // Convert depth to range using the depth2range LUT
      float r = *(lutPtr++)*depthrow[col];
      const cv::Vec3b& in = inrow[col];
      cv::Vec3b& out = outrow[col];

      if (r < 1e-3)
        r = 1e10;

      for (int c = 0; c < 3; c++)
      {
        // Simplifying assumption: intensity ~ irradiance.
        // This is not really the case but a good enough approximation
        // for now (it would be better to use a proper Radiometric
        // Response Function).
        float e = std::exp(-r*this->dataPtr->attenuation[c]);
        out[c] = e*in[c] + (1.0f-e)*this->dataPtr->background[c];
      }
    }
  }
}

void UnderwaterCamera::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
  if (!_info.paused)
  {
    rclcpp::spin_some(this->ros_node_);

    if (_info.iterations % 1000 == 0)
    {
      gzmsg << "dave_gz_sensor_plugins::UnderwaterCamera::PostUpdate" << std::endl;
    }
  }
}

}  // namespace dave_gz_sensor_plugins
