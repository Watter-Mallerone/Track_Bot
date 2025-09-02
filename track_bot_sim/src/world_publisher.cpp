#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("world_publisher");

  auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  const std::string model_name = "track_bot";
  const std::string relative_entity_name = "world";

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      rclcpp::spin_some(node);
      rate.sleep();
      continue;
    }

    auto req = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    req->name = model_name;
    req->reference_frame = relative_entity_name;

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(1))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto res = future.get();
      if (res->success) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id  = "base_link";

        t.transform.translation.x = res->state.pose.position.x;
        t.transform.translation.y = res->state.pose.position.y;
        t.transform.translation.z = res->state.pose.position.z;
        t.transform.rotation      = res->state.pose.orientation;

        tf_broadcaster->sendTransform(t);
      } else {
        RCLCPP_ERROR(node->get_logger(), "GetEntityState returned success=false");
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call /gazebo/get_entity_state");
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
