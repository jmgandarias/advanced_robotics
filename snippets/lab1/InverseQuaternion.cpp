tf2::Quaternion InverseQuaternion(const tf2::Quaternion &q)
{
    // This function computes the inverse of a quaternion q and returns the resulting quaternion.
    // The inverse of a quaternion q = (x, y, z, w) is given by:
    // q_inv = (-x, -y, -z, w) / (x^2 + y^2 + z^2 + w^2)
    // where the numerator is the conjugate of the quaternion and the denominator is the norm squared of the quaternion.

    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();

    double norm = sqrt(x * x + y * y + z * z + w * w);

    if (norm == 0.0)
    {
        throw std::runtime_error("Cannot compute inverse of a zero-norm quaternion");
    }

    double x_inv = -x / norm;
    double y_inv = -y / norm;
    double z_inv = -z / norm;
    double w_inv = w / norm;

    return tf2::Quaternion(x_inv, y_inv, z_inv, w_inv);
}