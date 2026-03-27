if (trajectory_msg.points.empty())
{
    std::fprintf(stderr, "No valid trajectory points were generated. Nothing will be published.\n");
    return 1;
}

while (pub->get_subscription_count() == 0 && rclcpp::ok())
{
    std::fprintf(stderr, "Waiting for /r6bot_controller/joint_trajectory subscriber...\n");
    rclcpp::sleep_for(std::chrono::milliseconds(200));
}

trajectory_msg.header.stamp = node->now() + rclcpp::Duration::from_seconds(0.2);
std::fprintf(stderr, "Publishing %zu trajectory points.\n", trajectory_msg.points.size());
pub->publish(trajectory_msg);
while (rclcpp::ok())
{
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
}
}

return 0;
}