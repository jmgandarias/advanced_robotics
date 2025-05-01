/*

Author: Juan M. Gandarias (http://jmgandarias.com)
email: jmgandarias@uma.es


This script computes the dynamic model of the RR UMA manipulator according to the equations of motion:

M(q)q'' + C(q,q')q' + Fbq' + g(q) = tau + tau_ext

then: q'' = M^(-1)[tau - C(q,q')q' - Fbq' - g(q) + tau_ext]

we assume tau_ext is given from the measures of an F/T sensor in the EE, then:

q'' = M^(-1)[tau - C(q,q')q' - Fbq' - g(q) + J^T(q)Fext]

Inputs: joint_torques, external_wrenches

Output: joint_states(joint_positions, joint_velocities) and joint_accelerations

*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono;

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

        // Create subscription to joint_torques
        joint_torques_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_torques", 1, std::bind(&ManipulatorDynamicsNode::joint_torques_callback, this, std::placeholders::_1));

        // Create subscription to joint_torques
        external_wrenches_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "external_wrenches", 1, std::bind(&ManipulatorDynamicsNode::external_wrenches_callback, this, std::placeholders::_1));

        // Create publishers for joint acceleration
        publisher_acceleration_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_accelerations", 1);

        // Create publisher for joint state
        publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

        // Set the timer callback at a period (in milliseconds, multiply it by 1000)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)), std::bind(&ManipulatorDynamicsNode::timer_callback, this));
    }

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

private:
    // Subscription callback - when a new message arrives, updates joint_torques_
    void joint_torques_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        joint_torques_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
    }

    // Subscription callback - when a new message arrives, updates external_wrenches_
    void external_wrenches_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        auto forces = msg->force;
        // This change of coordinates is based on how the dynamic model is defined in the 2D plane and the EE frame is defined in the 3D plane
        external_wrenches_(0) = forces.x;
        external_wrenches_(1) = forces.y;
    }

    // Method to calculate joint acceleration
    Eigen::VectorXd calculate_acceleration()
    {

        
        // Initialize M, C, Fb, g_vec, J, and tau_ext

        // Initialize q1, q2, q_dot1, and q_dot2

        // Placeholder calculations for M, C, Fb, g, and tau_ext
        // Calculate matrix M

        // Calculate vector C (C is 2x1 because it already includes q_dot)

        // Calculate Fb matrix

        // Calculate g_vect

        // Calculate J

        // Calculate tau_ext

        // Calculate joint accelerations using the dynamic model: q'' = M^(-1)[tau - C(q,q')q' - Fbq' - g(q) + tau_ext]
        Eigen::VectorXd q_ddot(2);
        q_ddot << 0, 0;

        // Return joint accelerations
        return q_ddot;
    }

    // Method to calculate joint velocity
    Eigen::VectorXd calculate_velocity()
    {
        // Placeholder for velocity calculation
        // Integrate velocity over the time step (elapsed_time_)
        Eigen::VectorXd q_dot(2);
        q_dot << 0, 0;

        return q_dot;
    }

    // Method to calculate joint position
    Eigen::VectorXd calculate_position()
    {
        // Placeholder for position calculation
        // Integrate position over the time step (elapsed_time_)
        Eigen::VectorXd q(2);
        q << 0, 0;
        
        return q;
    }

    // Method to publish the joint data
    void publish_data()
    {
        // publish joint acceleration
        auto acceleration_msg = std_msgs::msg::Float64MultiArray();
        acceleration_msg.data.assign(joint_accelerations_.data(), joint_accelerations_.data() + joint_accelerations_.size());
        publisher_acceleration_->publish(acceleration_msg);

        // publish joint state
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name = {"joint_1", "joint_2"}; // Replace with actual joint names
        joint_state_msg.position = {joint_positions_(0), joint_positions_(1)};
        joint_state_msg.velocity = {joint_velocities_(0), joint_velocities_(1)};
        publisher_joint_state_->publish(joint_state_msg);
    }

    // Member variables
    // Publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torques_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr external_wrenches_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_acceleration_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Joint variables
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd joint_accelerations_;
    Eigen::VectorXd joint_torques_;
    Eigen::VectorXd external_wrenches_;

    // dynamic parameters variables
    double m1_;
    double m2_;
    double l1_;
    double l2_;
    double b1_;
    double b2_;
    double g_;

    // Variable to store the previous callback time and elapsed time
    time_point<high_resolution_clock> previous_time_;
    double elapsed_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorDynamicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}