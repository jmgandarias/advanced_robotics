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
