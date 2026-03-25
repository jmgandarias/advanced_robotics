// Author: Juan M. Gandarias
// email: jmgandarias@uma.es
// web: https://jmgandarias.com
//
// Inspired from the ros2_control Development Team demos

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <string>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

Eigen::Matrix4d ParsePoseMatrix(const YAML::Node &root, const std::string &key)
{
    // This function parses a 4x4 matrix from a YAML node given a specific key word.
    if (!root[key] || !root[key].IsSequence() || root[key].size() != 4)
    {
        throw std::runtime_error("YAML key '" + key + "' must be a 4x4 matrix sequence");
    }

    Eigen::Matrix4d pose;
    for (int row = 0; row < 4; ++row)
    {
        const YAML::Node row_node = root[key][row];
        if (!row_node.IsSequence() || row_node.size() != 4)
        {
            throw std::runtime_error("YAML key '" + key + "' must contain rows of length 4");
        }
        for (int col = 0; col < 4; ++col)
        {
            pose(row, col) = row_node[col].as<double>();
        }
    }

    return pose;
}

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

// To be completed in Exercise 1
std::pair<tf2::Vector3, tf2::Quaternion> PoseInterpolation(
    const Eigen::Matrix4d &start_pose,
    const Eigen::Matrix4d &end_pose,
    double lambda)
{
    tf2::Vector3 p_interp;    // Placeholder for the interpolated position
    tf2::Quaternion q_interp; // Placeholder for the interpolated quaternion

    return {p_interp, q_interp};
}

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("send_trajectory");
    auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/r6bot_controller/joint_trajectory", 10);

    // get robot description
    auto robot_param = rclcpp::Parameter();
    node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    node->get_parameter("robot_description", robot_param);
    auto robot_description = robot_param.as_string();

    // create kinematic chain
    KDL::Tree robot_tree;
    KDL::Chain chain;
    kdl_parser::treeFromString(robot_description, robot_tree);
    robot_tree.getChain("base_link", "tool0", chain);

    auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
    // create KDL solvers
    auto fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);
    auto ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(
        chain, *fk_solver_, *ik_vel_solver_, 100, 1e-6);

    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = node->now();
    for (size_t i = 0; i < chain.getNrOfSegments(); i++)
    {
        auto joint = chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::Fixed)
        {
            trajectory_msg.joint_names.push_back(joint.getName());
        }
    }

    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
    trajectory_point_msg.positions.resize(chain.getNrOfJoints());
    trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

    std::string poses_yaml_path;
    node->declare_parameter<std::string>("poses_yaml", "");
    node->get_parameter("poses_yaml", poses_yaml_path);

    if (poses_yaml_path.empty())
    {
        poses_yaml_path =
            ament_index_cpp::get_package_share_directory("cartesian_trajectory_planning") +
            "/config/poses.yaml";
    }

    Eigen::Matrix4d pose0;
    Eigen::Matrix4d pose1;
    Eigen::Matrix4d pose2;
    try
    {
        const YAML::Node poses_root = YAML::LoadFile(poses_yaml_path);
        pose0 = ParsePoseMatrix(poses_root, "pose0");
        pose1 = ParsePoseMatrix(poses_root, "pose1");
        pose2 = ParsePoseMatrix(poses_root, "pose2");
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "Failed to load poses YAML '%s': %s\n", poses_yaml_path.c_str(), e.what());
        return 1;
    }

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

    // Exercise 2 : Cartesian trajectory generation
    int tau = 1;
    int T = 10;
    bool exercise_2 = false; // Set to true to execute Exercise 2

    if (exercise_2)
    {
        trajectory_msg.points.clear();
        const double sample_time = 0.1;
        int point_index = 0;

        const std::string package_name = "cartesian_trajectory_planning";
        const std::string package_share_dir =
            ament_index_cpp::get_package_share_directory(package_name);

        std::string output_base_dir = package_share_dir;
        const std::string install_suffix =
            "/install/" + package_name + "/share/" + package_name;
        const std::size_t suffix_pos = package_share_dir.rfind(install_suffix);
        if (suffix_pos != std::string::npos)
        {
            const std::string workspace_root = package_share_dir.substr(0, suffix_pos);
            const std::string source_candidate = workspace_root + "/src/" + package_name;

            struct stat source_candidate_stat;
            if (::stat(source_candidate.c_str(), &source_candidate_stat) == 0 &&
                S_ISDIR(source_candidate_stat.st_mode))
            {
                output_base_dir = source_candidate;
            }
        }

        const std::string output_dir = output_base_dir + "/experiment_data";

        if (::mkdir(output_dir.c_str(), 0755) != 0 && errno != EEXIST)
        {
            std::fprintf(stderr, "Failed to create output directory '%s': %s.\n", output_dir.c_str(), std::strerror(errno));
            return 1;
        }

        const auto now = std::chrono::system_clock::now();
        const std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;
        localtime_r(&now_time_t, &local_tm);

        std::ostringstream filename_builder;
        filename_builder << "data_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".csv";
        const std::string output_csv = output_dir + "/" + filename_builder.str();
        std::ofstream csv_file(output_csv, std::ios::out | std::ios::trunc);
        if (!csv_file.is_open())
        {
            std::fprintf(stderr, "Failed to open CSV output file '%s'.\n", output_csv.c_str());
            return 1;
        }

        csv_file << "t,X,Y,Z,roll,pitch,yaw\n";
        csv_file << std::fixed << std::setprecision(9);

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

        if (trajectory_msg.points.empty())
        {
            std::fprintf(stderr, "No valid trajectory points were generated. Nothing will be published.\n");
            return 1;
        }

        while (pub->get_subscription_count() == 0 && rclcpp::ok())
        {
            std::fprintf(stderr, "Waiting for /r6bot_controller/joint_trajectory subscriber...\n");
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        trajectory_msg.header.stamp = node->now() + rclcpp::Duration::from_seconds(0.2);
        std::fprintf(stderr, "Publishing %zu trajectory points.\n", trajectory_msg.points.size());
        pub->publish(trajectory_msg);
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return 0;
}
