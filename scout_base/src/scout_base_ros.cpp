/*
 * scout_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "scout_base/scout_base_ros.hpp"

#include "scout_base/scout_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
ScoutBaseRos::ScoutBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", "can0");

  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("odom_topic_name", "odom");

  this->declare_parameter("is_scout_mini", false);
  this->declare_parameter("is_omni_wheel", false);
  this->declare_parameter("auto_reconnect", true);

  this->declare_parameter("simulated_robot", false);
  this->declare_parameter("control_rate", 50);

  LoadParameters();
}

void ScoutBaseRos::LoadParameters() {
  this->get_parameter<std::string>("port_name", port_name_);

  this->get_parameter<std::string>("odom_frame", odom_frame_);
  this->get_parameter<std::string>("base_frame", base_frame_);
  this->get_parameter<std::string>("odom_topic_name", odom_topic_name_);

  this->get_parameter<bool>("is_scout_mini", is_scout_mini_);
  this->get_parameter<bool>("is_omni_wheel", is_omni_wheel_);
  this->get_parameter<bool>("auto_reconnect", auto_reconnect_);

  this->get_parameter<bool>("simulated_robot", simulated_robot_);
  this->get_parameter<int>("control_rate", sim_control_rate_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Loading parameters: ");
  RCLCPP_INFO_STREAM(this->get_logger(), "- port name: " << port_name_);
  RCLCPP_INFO_STREAM(this->get_logger(), "- odom frame name: " << odom_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "- base frame name: " << base_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "- odom topic name: " << odom_topic_name_);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "- is scout mini: " << std::boolalpha << is_scout_mini_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "- is omni wheel: " << std::boolalpha << is_omni_wheel_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "- auto reconnect: " << std::boolalpha << auto_reconnect_);

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "- simulated robot: " << std::boolalpha << simulated_robot_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "- sim control rate: " << sim_control_rate_);
  RCLCPP_INFO_STREAM(this->get_logger(), "----------------------------");
}

bool ScoutBaseRos::Initialize() {
  if (is_scout_mini_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot base: Scout Mini");
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot base: Scout");
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Detected protocol: AGX_V1");
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V1,
                                              is_scout_mini_);
        if (is_scout_mini_) {
          RCLCPP_INFO_STREAM(
              this->get_logger(),
              "Creating interface for Scout Mini with AGX_V1 Protocol");
        } else {
          RCLCPP_INFO_STREAM(
              this->get_logger(),
              "Creating interface for Scout with AGX_V1 Protocol");
        }
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Creating interface for Scout Mini Omni with AGX_V1 Protocol");
      }
    } else if (proto == ProtocolVersion::AGX_V2) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Detected protocol: AGX_V2");
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V2,
                                              is_scout_mini_);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Creating interface for Scout with AGX_V2 Protocol");
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Creating interface for Scout Mini Omni with AGX_V2 Protocol");
      }
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Detected protocol: UNKONWN");
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void ScoutBaseRos::Stop() { keep_running_ = false; }

void ScoutBaseRos::Run() {
  // instantiate a ROS messenger
  if (!is_omni_) {
    std::unique_ptr<ScoutMessenger<ScoutRobot>> messenger =
        std::unique_ptr<ScoutMessenger<ScoutRobot>>(
            new ScoutMessenger<ScoutRobot>(robot_, this));

    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);
    messenger->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {
        robot_->EnableCommandedMode();
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Using CAN bus to talk with the robot");
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Failed to connect to the robot CAN bus");
        return;
      }
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Please check the specified port name is a CAN port");
      return;
    }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    AgxControlMode robot_control;
    keep_running_ = true;
    rclcpp::Rate rate(50);
    while (keep_running_) {
      messenger->PublishStateToROS();
      robot_control = robot_->GetRobotState().system_state.control_mode;
      if (auto_reconnect_ && robot_control == CONTROL_MODE_STANDBY) {
        robot_->EnableCommandedMode();
      }
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  } else {
    std::unique_ptr<ScoutMessenger<ScoutMiniOmniRobot>> messenger =
        std::unique_ptr<ScoutMessenger<ScoutMiniOmniRobot>>(
            new ScoutMessenger<ScoutMiniOmniRobot>(omni_robot_, this));

    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);
    messenger->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (omni_robot_->Connect(port_name_)) {
        omni_robot_->EnableCommandedMode();
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Using CAN bus to talk with the robot");
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Failed to connect to the robot CAN bus");
        return;
      }
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Please check the specified port name is a CAN port");
      return;
    }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    rclcpp::Rate rate(50);
    keep_running_ = true;
    while (keep_running_) {
      messenger->PublishStateToROS();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  }
}
}  // namespace westonrobot
