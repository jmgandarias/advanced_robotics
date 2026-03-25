tf2::Quaternion rot2Quat(const Eigen::Matrix3d &R, int m = 1)
{
    // This function converts a rotation matrix R to a quaternion representation.
    int M = (m >= 0) ? 1 : -1;
    double w = M * std::sqrt(R(0, 0) + R(1, 1) + R(2, 2) + 1.0) / 2.0;

    double x, y, z;

    if (std::abs(w) > 1e-3)
    {
        x = (R(2, 1) - R(1, 2)) / (4.0 * w);
        y = (R(0, 2) - R(2, 0)) / (4.0 * w);
        z = (R(1, 0) - R(0, 1)) / (4.0 * w);
    }
    else
    {
        w = 0.0;
        x = M * std::sqrt((R(0, 0) + 1.0) / 2.0);
        y = M * std::sqrt((R(1, 1) + 1.0) / 2.0);
        z = M * std::sqrt((R(2, 2) + 1.0) / 2.0);
    }

    return tf2::Quaternion(x, y, z, w);
}