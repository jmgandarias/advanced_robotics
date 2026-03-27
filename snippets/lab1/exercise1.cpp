// Exercise 1 : Cartesian interpolation
const auto [p0, q0] = PoseInterpolation(pose0, pose1, 0.0);
const auto [p1, q1] = PoseInterpolation(pose0, pose1, 1.0);
const auto [p2, q2] = PoseInterpolation(pose1, pose2, 0.0);
const auto [p3, q3] = PoseInterpolation(pose1, pose2, 1.0);

// Check the results of the interpolation
printf("p0: %f, %f, %f\n", p0.x(), p0.y(), p0.z());
printf("q0: %f, %f, %f, %f\n",
       q0.x(), q0.y(), q0.z(), q0.w());
printf("p1: %f, %f, %f\n", p1.x(), p1.y(), p1.z());
printf("q1: %f, %f, %f, %f\n",
       q1.x(), q1.y(), q1.z(), q1.w());
printf("p2: %f, %f, %f\n", p2.x(), p2.y(), p2.z());
printf("q2: %f, %f, %f, %f\n",
       q2.x(), q2.y(), q2.z(), q2.w());
printf("p3: %f, %f, %f\n", p3.x(), p3.y(), p3.z());
printf("q3: %f, %f, %f, %f\n",
       q3.x(), q3.y(), q3.z(), q3.w());