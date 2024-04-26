/*
 * scout_base_ros.hpp
 *
 * Created on: Oct 15, 2021 14:31
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_BASE_ROS_HPP
#define SCOUT_BASE_ROS_HPP

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace westonrobot {
class ScoutBaseRos : public rclcpp::Node {
 public:
  ScoutBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void Stop();

 private:
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_scout_mini_ = false;
  bool is_omni_wheel_ = false;
  bool auto_reconnect_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  bool is_omni_ = false;
  std::shared_ptr<ScoutRobot> robot_;
  std::shared_ptr<ScoutMiniOmniRobot> omni_robot_;

  std::atomic<bool> keep_running_;

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_ROS_HPP */
