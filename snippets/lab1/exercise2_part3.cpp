// Loop over the time range from -T to T with a step of sample_time and compute the interpolated Cartesian pose at each time step
for (double t = -T; t <= T + 1e-9; t += sample_time)
{
    const auto [p_interp, q_interp] = ComputeNextCartesianPose(pose0, pose1, pose2, tau, T, t);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);

    csv_file << t << ","
             << p_interp.x() << "," << p_interp.y() << "," << p_interp.z() << ","
             << roll << "," << pitch << "," << yaw << "\n";

    const KDL::Frame desired_ee_pose(
        KDL::Rotation::Quaternion(q_interp.x(), q_interp.y(), q_interp.z(), q_interp.w()),
        KDL::Vector(p_interp.x(), p_interp.y(), p_interp.z()));

    KDL::JntArray next_joint_positions(chain.getNrOfJoints());
    const int ik_status = ik_pos_solver_->CartToJnt(joint_positions, desired_ee_pose, next_joint_positions);
    if (ik_status < 0)
    {
        std::fprintf(
            stderr,
            "IK failed at t=%.3f with error code %d. Skipping point.\n",
            t, ik_status);
        continue;
    }

    std::memcpy(
        trajectory_point_msg.positions.data(), next_joint_positions.data.data(),
        trajectory_point_msg.positions.size() * sizeof(double));

    std::fill(trajectory_point_msg.velocities.begin(), trajectory_point_msg.velocities.end(), 0.0);

    const double elapsed = static_cast<double>(point_index + 1) * sample_time;
    trajectory_point_msg.time_from_start.sec = static_cast<int32_t>(elapsed);
    trajectory_point_msg.time_from_start.nanosec = static_cast<uint32_t>(
        (elapsed - static_cast<double>(trajectory_point_msg.time_from_start.sec)) * 1e9);

    trajectory_msg.points.push_back(trajectory_point_msg);
    joint_positions = next_joint_positions;
    point_index++;
}