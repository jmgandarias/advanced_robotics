/*

Author: Juan M. Gandarias (http://jmgandarias.com)
email: jmgandarias@uma.es

This script calculates the torque to cancellate the gracvity dynamic effects

tau = g(q)

Inputs: desired_joint_accelerations, joint_state(joint_positions, joint_velocities)

Output: joint_torques

*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>

class DynamicsCancellationNode : public rclcpp::Node
{
public:
    DynamicsCancellationNode()
        : Node("gravity_compensation_node"),
          joint_positions_(Eigen::VectorXd::Zero(2)),
          joint_torques_(Eigen::VectorXd::Zero(2))
    {
        // Frequency initialization
        this->declare_parameter<double>("frequency", 1000.0);

        // Dynamics parameters initialization
        this->declare_parameter<double>("m2", 1.0);
        this->declare_parameter<double>("m1", 1.0);
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

        // Set initial joint state
        joint_positions_ = Eigen::VectorXd::Map(this->get_parameter("q0").as_double_array().data(), 2);

        // Create subscription to joint_torques
        subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&DynamicsCancellationNode::joint_states_callback, this, std::placeholders::_1));

        // Create publishers for joint torque
        publisher_joint_torques_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_torques", 1);

        // Set the timer callback at a period (in milliseconds, multiply it by 1000)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)), std::bind(&DynamicsCancellationNode::timer_callback, this));
    }

    // Timer callback - when there is a timer callback, computes the new joint acceleration, velocity and position and publishes them
    void timer_callback()
    {
        // Calculate torque to cancel the dynamic effects
        joint_torques_ = gravity_compensation();

        // Publish data
        publish_data();
    }

private:
    // joint_states subscription callback - when a new message arrives, updates the dynamics cancellation and publishes teh joint_torques_
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        // Assuming the joint names are "joint_1" and "joint_2"
        auto joint1_index = std::find(msg->name.begin(), msg->name.end(), "joint_1") - msg->name.begin();
        auto joint2_index = std::find(msg->name.begin(), msg->name.end(), "joint_2") - msg->name.begin();

        if (static_cast<std::vector<std::string>::size_type>(joint1_index) < msg->name.size() && 
            static_cast<std::vector<std::string>::size_type>(joint2_index) < msg->name.size())
        {
            joint_positions_(0) = msg->position[joint1_index];
            joint_positions_(1) = msg->position[joint2_index];
        }
    }

    // Method to calculate the desired joint torques
    Eigen::VectorXd gravity_compensation()
    {
        // Placeholder for calculate the commanded torques
        // Calculate the control torque to compensate only for gravity effects: tau = g(q)

        // Calculate g_vect

        // // Calculate desired torque
        Eigen::VectorXd torque(2);
        torque << 0, 0;

        return torque;
    }

    // Method to publish the joint data
    void publish_data()
    {
        // publish joint torque
        auto joint_torques_msg = std_msgs::msg::Float64MultiArray();
        joint_torques_msg.data.assign(joint_torques_.data(), joint_torques_.data() + joint_torques_.size());
        publisher_joint_torques_->publish(joint_torques_msg);
    }

    // Member variables
    // Publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_joint_torques_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Joint variables
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_torques_;

    // dynamic parameters variables
    double m1_;
    double m2_;
    double l1_;
    double l2_;
    double b1_;
    double b2_;
    double g_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicsCancellationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}