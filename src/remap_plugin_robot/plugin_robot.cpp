// Copyright 2025 PAL Robotics, S.L.
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>

#include "remap_plugin_robot/plugin_robot.hpp"

namespace remap
{
namespace plugins
{
PluginRobot::PluginRobot()
: SemanticPlugin() {}

PluginRobot::PluginRobot(
  std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
  std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register)
: SemanticPlugin(semantic_map, regions_register) {}

PluginRobot::~PluginRobot()
{
  // semantic_map_->removeRegion("gaze_" + robot_name_, *regions_register_);

  semantic_map_.reset();
  regions_register_.reset();
}

void PluginRobot::representGaze(
  const geometry_msgs::msg::TransformStamped & gaze_transform,
  const std::string & gaze_id)
{
  auto gaze_translation = gaze_transform.transform.translation;
  auto gaze_rotation = gaze_transform.transform.rotation;

  tf2::Quaternion q_A_to_B(
    gaze_rotation.x,
    gaze_rotation.y,
    gaze_rotation.z,
    gaze_rotation.w);
  q_A_to_B.normalize();

  tf2::Vector3 axis_in_B(0.0, 0.0, 1.0);
  auto axis_in_A = tf2::quatRotate(q_A_to_B, axis_in_B);

  openvdb::Vec3d gaze_direction(
    axis_in_A.getX(),
    axis_in_A.getY(),
    axis_in_A.getZ());
  openvdb::Vec3d gaze_origin(
    gaze_translation.x,
    gaze_translation.y,
    gaze_translation.z);
  semantic_map_->insertSemanticPyramid(
    fov_h_,
    fov_v_,
    2.0,
    gaze_direction,
    std::string("gaze_") + gaze_id,
    *regions_register_,
    gaze_origin,
    0.1);
}

void PluginRobot::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!fov_set_) {
    fov_h_ = 2 * std::atan2(msg->k[2], msg->k[0]);
    fov_v_ = 2 * std::atan2(msg->k[5], msg->k[4]);
    fov_set_ = true;
    camera_info_sub_.reset();
  }
}

void PluginRobot::initialize()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "PluginRobot initializing");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  fov_set_ = false;

  rclcpp::QoS camera_info_qos(1);
  camera_info_qos.best_effort();

  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};

  descriptor.description = "Robot name";
  node_ptr_->declare_parameter("plugin/robot/robot_name", "robot", descriptor);

  descriptor.description = "Robot gaze reference frame";
  node_ptr_->declare_parameter("plugin/robot/gaze_frame", "camera_optical_frame", descriptor);

  descriptor.description = "Robot presence reference frame";
  node_ptr_->declare_parameter("plugin/robot/presence_frame", "base_link", descriptor);

  robot_name_ = node_ptr_->get_parameter("plugin/robot/robot_name").as_string();
  robot_gaze_frame_ = node_ptr_->get_parameter("plugin/robot/gaze_frame").as_string();
  robot_presence_frame_ = node_ptr_->get_parameter("plugin/robot/presence_frame").as_string();
  robot_presence_frame_ = node_ptr_->get_parameter("plugin/robot/presence_frame").as_string();
  fixed_frame_ = node_ptr_->get_parameter("fixed_frame").as_string();

  this->pushFact(robot_name_ + " rdf:type Robot");

  camera_info_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", camera_info_qos,
    std::bind(&PluginRobot::cameraInfoCallback, this, std::placeholders::_1));
}

void PluginRobot::run()
{
  if (fov_set_) {
    semantic_map_->removeRegion("gaze_" + robot_name_, *regions_register_);

    geometry_msgs::msg::TransformStamped robot_gaze_transform;
    try {
      robot_gaze_transform = tf_buffer_->lookupTransform(
        fixed_frame_, robot_gaze_frame_, tf2::TimePointZero);
      representGaze(robot_gaze_transform, robot_name_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
  }

  // We activate one single voxel containing the name of the robot
  // to represent where it is located
  semantic_map_->removeRegion(robot_name_, *regions_register_);
  geometry_msgs::msg::TransformStamped robot_presence_transform;

  try {
    robot_presence_transform = tf_buffer_->lookupTransform(
      fixed_frame_, robot_presence_frame_, tf2::TimePointZero);
    semantic_map_->insertVoxel(
      robot_presence_transform.transform.translation.x,
      robot_presence_transform.transform.translation.y,
      robot_presence_transform.transform.translation.z,
      robot_name_,
      *regions_register_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Could not transform: %s", ex.what());
    return;
  }
}

void PluginRobot::storeEntitiesRelationships(
  std::map<std::string, std::map<std::string, std::string>> relationships_matrix)
{
  (void) relationships_matrix;

  auto in_fov_entities = regions_register_->getCoexistentEntities("gaze_" + robot_name_);
  auto presence_entities = regions_register_->getCoexistentEntities(robot_name_);

  std::vector<std::string> new_facts;
  std::vector<std::string> old_facts;

  for (const auto & object : in_fov_entities) {
    if (in_fov_entities_.find(object) == in_fov_entities_.end()) {
      new_facts.push_back(robot_name_ + " oro:sees " + object);
    }
  }
  for (const auto & object : in_fov_entities_) {
    if (in_fov_entities.find(object) == in_fov_entities.end()) {
      old_facts.push_back(robot_name_ + " oro:sees " + object);
    }
  }

  for (const auto & room : presence_entities) {
    if (presence_entities_.find(room) == presence_entities_.end()) {
      new_facts.push_back(robot_name_ + " oro:isIn " + room);
    }
  }
  for (const auto & room : presence_entities_) {
    if (presence_entities.find(room) == presence_entities.end()) {
      old_facts.push_back(robot_name_ + " oro:isIn " + room);
    }
  }

  if (new_facts.size() > 0) {
    this->revisePushFacts(new_facts);
  }
  if (old_facts.size() > 0) {
    this->reviseRemoveFacts(old_facts);
  }

  in_fov_entities_ = in_fov_entities;
  presence_entities_ = presence_entities;
}
}  // namespace plugins
}  // namespace remap

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(remap::plugins::PluginRobot, remap::plugins::PluginBase)
