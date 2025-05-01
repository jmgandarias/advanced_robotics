// Method to publish the joint data
void publish_data()
{
    // publish joint acceleration
    auto acceleration_msg = std_msgs::msg::Float64MultiArray();
    acceleration_msg.data.assign(joint_accelerations_.data(), joint_accelerations_.data() + joint_accelerations_.size());
    publisher_acceleration_->publish(acceleration_msg);

    // publish joint state
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = {"joint_1", "joint_2"}; // Replace with actual joint names
    joint_state_msg.position = {joint_positions_(0), joint_positions_(1)};
    joint_state_msg.velocity = {joint_velocities_(0), joint_velocities_(1)};
    publisher_joint_state_->publish(joint_state_msg);
}