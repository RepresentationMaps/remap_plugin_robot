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

#ifndef REMAP_PLUGIN_ROBOT__PLUGIN_ROBOT_HPP_
#define REMAP_PLUGIN_ROBOT__PLUGIN_ROBOT_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <remap_plugin_base/plugin_base.hpp>
#include <remap_plugin_base/semantic_plugin.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

namespace remap
{
namespace plugins
{
class PluginRobot : public SemanticPlugin
{
private:
  std::vector<std::string> regions_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string fixed_frame_;

  std::string robot_name_;
  std::string robot_gaze_frame_;
  std::string robot_presence_frame_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  float fov_h_;
  float fov_v_;
  bool fov_set_;

  std::unordered_set<std::string> in_fov_entities_;
  std::unordered_set<std::string> presence_entities_;

  void representGaze(
    const geometry_msgs::msg::TransformStamped & gaze_transform,
    const std::string & gaze_id);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

public:
  PluginRobot();
  PluginRobot(
    std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
    std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register);
  ~PluginRobot();
  void run() override;
  void initialize() override;

  void storeEntitiesRelationships(
    std::map<std::string, std::map<std::string, std::string>> relationships_matrix);
};
}    // namespace plugins
}  // namespace remap
#endif  // REMAP_PLUGIN_ROBOT__PLUGIN_ROBOT_HPP_
