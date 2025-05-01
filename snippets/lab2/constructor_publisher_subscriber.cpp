 // Create subscription to joint_torques
 joint_torques_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_torques", 1, std::bind(&ManipulatorDynamicsNode::joint_torques_callback, this, std::placeholders::_1));

// Create subscription to external wrenches
external_wrenches_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
    "external_wrenches", 1, std::bind(&ManipulatorDynamicsNode::external_wrenches_callback, this, std::placeholders::_1));

// Create publisher for joint acceleration
publisher_acceleration_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_accelerations", 1);

// Create publisher for joint state
publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

// Set the timer callback at a period (in milliseconds, multiply it by 1000)
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / frequency)), std::bind(&ManipulatorDynamicsNode::timer_callback, this));
}
