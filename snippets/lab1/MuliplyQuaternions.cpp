tf2::Quaternion MuliplyQuaternions(const tf2::Quaternion &q1, const tf2::Quaternion &q2)
{
    // This function multiplies two quaternions q1 and q2 and returns the resulting quaternion.
    // The multiplication is defined as:
    // q_result = q1 * q2
    // where q1 and q2 are represented as (x, y, z, w) and the multiplication is performed using the Hamilton product.

    double x1 = q1.x();
    double y1 = q1.y();
    double z1 = q1.z();
    double w1 = q1.w();

    double x2 = q2.x();
    double y2 = q2.y();
    double z2 = q2.z();
    double w2 = q2.w();

    double x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    double y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    double z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    double w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

    return tf2::Quaternion(x_result, y_result, z_result, w_result);
}