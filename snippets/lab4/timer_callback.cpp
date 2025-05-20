// Timer callback - when there is a timer callback, computes the new joint acceleration, velocity and position and publishes them
    void timer_callback()
    {

        if (!joint_states_received_ || !external_wrench_received_ || !equilibrium_pose_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for all inputs: joint_states [%s], external_wrenches [%s], equilibrium_pose [%s]",
                                 joint_states_received_ ? "OK" : "MISSING",
                                 external_wrench_received_ ? "OK" : "MISSING",
                                 equilibrium_pose_received_ ? "OK" : "MISSING");
            return;
        }

        cartesian_pose_ = forward_kinematics();                                 // Calculate cartesian pose
        update_jacobians();                                                     // Update jacobian and jacobian derivative
        cartesian_velocities_ = differential_kinematics();                      // Calculate Cartesian velocity with first-order differental kinematics
        desired_cartesian_accelerations_ = impedance_controller();              // Calculate desired cartesian accelerations with impedance controller
        desired_joint_accelerations_ = calculate_desired_joint_accelerations(); // Calculate the desired_joint_accelerations

        if ((desired_joint_accelerations_.array().isNaN()).any())
        {
            RCLCPP_ERROR(this->get_logger(), "Computed NaN in desired_joint_accelerations. Skipping publish.");
            return;
        }

        // Publish data
        publish_data();
    }