private:
    // Subscription callback - when a new message arrives, updates joint_torques_
    void joint_torques_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        joint_torques_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
    }

    // Subscription callback - when a new message arrives, updates external_wrenches_
    void external_wrenches_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        auto forces = msg->force;
        external_wrenches_(0) = forces.x;
        external_wrenches_(1) = forces.y;
    }