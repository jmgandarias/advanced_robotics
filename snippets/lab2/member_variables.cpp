// Member variables
    // Publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torques_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr external_wrenches_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_acceleration_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Joint variables
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd joint_accelerations_;
    Eigen::VectorXd joint_torques_;
    Eigen::VectorXd external_wrenches_;

    // dynamic parameters variables
    double m1_;
    double m2_;
    double l1_;
    double l2_;
    double b1_;
    double b2_;
    double g_;

    // Variable to store the previous callback time and elapsed time
    time_point<high_resolution_clock> previous_time_;
    double elapsed_time_;
};