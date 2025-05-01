class ManipulatorDynamicsNode : public rclcpp::Node
{
public:
    ManipulatorDynamicsNode()
        : Node("manipulator_dynamics_node"),
          joint_positions_(Eigen::VectorXd::Zero(2)),
          joint_velocities_(Eigen::VectorXd::Zero(2)),
          joint_accelerations_(Eigen::VectorXd::Zero(2)),
          joint_torques_(Eigen::VectorXd::Zero(2)),
          external_wrenches_(Eigen::VectorXd::Zero(2)),
          previous_time_(high_resolution_clock::now())
    {
        // Frequency initialization
        this->declare_parameter<double>("frequency", 1000.0);

        // Dynamics parameters initialization
        this->declare_parameter<double>("m1", 1.0);
        this->declare_parameter<double>("m2", 1.0);
        this->declare_parameter<double>("l1", 1.0);
        this->declare_parameter<double>("l2", 1.0);
        this->declare_parameter<double>("b1", 1.0);
        this->declare_parameter<double>("b2", 1.0);
        this->declare_parameter<double>("g", 9.81);
        this->declare_parameter<std::vector<double>>("q0", {0, 0});

        // Get frequency [Hz] parameter and compute period [s]
        double frequency = this->get_parameter("frequency").as_double();

        // Get dynamic parameters
        m1_ = this->get_parameter("m1").as_double();
        m2_ = this->get_parameter("m2").as_double();
        l1_ = this->get_parameter("l1").as_double();
        l2_ = this->get_parameter("l2").as_double();
        g_ = this->get_parameter("g").as_double();
        b1_ = this->get_parameter("b1").as_double();
        b2_ = this->get_parameter("b2").as_double();

        // Set initial joint position
        joint_positions_ = Eigen::VectorXd::Map(this->get_parameter("q0").as_double_array().data(), 2);