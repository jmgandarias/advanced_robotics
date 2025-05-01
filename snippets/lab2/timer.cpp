// Timer callback - when there is a timer callback, computes the new joint acceleration, velocity and position and publishes them
void timer_callback()
{
// Get the actual elapsed time
auto current_time = high_resolution_clock::now();
elapsed_time_ = duration_cast<duration<double>>(current_time - previous_time_).count();
previous_time_ = current_time;

// Calculate JointState
joint_accelerations_ = calculate_acceleration();
joint_velocities_ = calculate_velocity();
joint_positions_ = calculate_position();

// Publish data
publish_data();
}