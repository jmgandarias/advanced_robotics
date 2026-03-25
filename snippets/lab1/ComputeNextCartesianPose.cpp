// To be completed in Exercise 2
std::pair<tf2::Vector3, tf2::Quaternion> ComputeNextCartesianPose(
    const Eigen::Matrix4d &pose_0,
    const Eigen::Matrix4d &pose_1,
    const Eigen::Matrix4d &pose_2,
    double tau,
    double T,
    double t)
{
    // Check if t is within the valid range of [-T, T]
    if (t < -T || t > T)
    {
        throw std::out_of_range("Parameter t is outside [-T, T]");
    }

    const tf2::Vector3 p_interp; // Placeholder for the interpolated position
    tf2::Quaternion q_interp;    // Placeholder for the interpolated quaternion
    return {p_interp, q_interp};
}