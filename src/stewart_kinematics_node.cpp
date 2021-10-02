// Copyright (c) 2021 Conroy Cheers

#include "hexapod_kinematics/hexapod_kinematics.h"

#include <chrono>
#include <vector>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


constexpr int num_struts = 6;


class StewartNode : public rclcpp::Node
{
public:
  StewartNode()
  : Node("stewart_kinematics_node", "stewart"), config(hexkins::HexapodConfig())
  {
    for (int i = 0; i < num_struts; ++i) {
      std::stringstream topic_base;
      topic_base << "arm_" << i << "/";
      std::string topic = topic_base.str();

      auto pub = this->create_publisher<std_msgs::msg::Float32>(
        topic + "position_target", rclcpp::QoS(10).best_effort());
      strut_publishers.push_back(pub);

      auto sub = this->create_subscription<std_msgs::msg::Float32>(
        topic + "current_position",
        rclcpp::QoS(10).best_effort(),
        [this, i](std_msgs::msg::Float32::UniquePtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received %f for strut %d", msg->data, i);
        }
      );
      strut_subscribers.push_back(sub);

      auto free_move_srv_client = this->create_client<std_srvs::srv::SetBool>(
        topic + "set_free_move");
      strut_free_move_clients.push_back(free_move_srv_client);
    }

    pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10, std::bind(&StewartNode::pose_callback, this, _1));

    free_move_service = this->create_service<std_srvs::srv::SetBool>(
      "set_free_move",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        for (auto & client : this->strut_free_move_clients) {
          auto result = client->async_send_request(request);
          auto call_r = rclcpp::spin_until_future_complete(shared_from_this(), result, 1s);
          if (call_r != rclcpp::FutureReturnCode::SUCCESS) {
            // call failed, time to return
            response->success = false;
            return;
          }
        }
        response->success = true;
      });

    config.base_joints = {{
      {273.001, -55.310, 0},
      {88.601, 264.081, 0},
      {-88.601, 264.081, 0},
      {-273.001, -55.310, 0},
      {-184.400, -208.771, 0},
      {184.400, -208.771, 0},
    }};
    config.platform_joints = {{
      {79.748, 17.685, 20},
      {57.039, 60.229, 20},
      {-55.187, 60.229, 20},
      {-79.748, 17.685, 20},
      {-24.559, -77.912, 20},
      {24.559, -77.912, 20},
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
      auto message = std_msgs::msg::Float32();
      message.data = struts[i];
      RCLCPP_INFO(this->get_logger(), "Publishing on %d: '%f'", i, message.data);
      strut_publishers[i]->publish(message);
    }
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> strut_publishers;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> strut_subscribers;
  std::vector<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> strut_free_move_clients;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr free_move_service;

  hexkins::HexapodConfig config;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StewartNode>());
  rclcpp::shutdown();
  return 0;
}
