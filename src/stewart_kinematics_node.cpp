// Copyright (c) 2021 Conroy Cheers

#include "hexapod_kinematics/hexapod_kinematics.h"

#include <chrono>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


constexpr int num_struts = 6;


class StewartNode : public rclcpp::Node
{
public:
  StewartNode()
  : Node("stewart_kinematics_node"), config(hexkins::HexapodConfig())
  {
    for (int i = 0; i < num_struts; ++i) {
      std::stringstream topic_name;
      topic_name << "~/strut_" << i << "/target_position";
      std::string topic = topic_name.str();
      auto pub = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
      strut_publishers.push_back(pub);
    }

    pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", 10, std::bind(&StewartNode::pose_callback, this, _1));

    config.base_joints = {{
      {2.57940852063914, 0.797904557985617, 0},
      {1.98070987733051, 1.83488102661872, 0},
      {-1.98070987733051, 1.83488102661872, 0},
      {-2.57940852063914, 0.797904557985617, 0},
      {-0.598698643308632, -2.63278558460434, 0},
      {0.598698643308631, -2.63278558460434, 0},
    }};
    config.platform_joints = {{
      {0.955336489125606, -0.295520206661340, 0},
      {0.221740238262456, 0.975105772075681, 0},
      {-0.221740238262455, 0.975105772075681, 0},
      {-0.955336489125606, -0.295520206661339, 0},
      {-0.733596250863151, -0.679585565414341, 0},
      {0.733596250863150, -0.679585565414341, 0},
    }};
    config.kins_max_iterations = 1000;
    config.kins_fwd_max_retries = 20;
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    Eigen::Vector3d pos({
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z});
    Eigen::Quaterniond ori({
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z});
    auto struts = hexkins::inverse_kinematics(config, pos, ori);
    for (int i = 0; i < num_struts; ++i) {
      auto message = std_msgs::msg::Float64();
      message.data = struts[i];
      RCLCPP_INFO(this->get_logger(), "Publishing on %d: '%f'", i, message.data);
      strut_publishers[i]->publish(message);
    }
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> strut_publishers;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

  hexkins::HexapodConfig config;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StewartNode>());
  rclcpp::shutdown();
  return 0;
}
