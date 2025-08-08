#include "custom_thruster_manager/custom_thruster_manager.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace custom_thruster_manager
{

CustomThrusterManager::CustomThrusterManager(const rclcpp::NodeOptions& options) : Node("custom_thruster_manager", options)
{
    declareParameters();
    loadParameters();
    
    // Setup custom TAM based on user requirements
    setupCustomTAM();
    
    // Setup publishers and subscribers
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "wrench", 10, std::bind(&CustomThrusterManager::wrenchCallback, this, std::placeholders::_1));
    
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_thrust", 10);
    
    // Create individual thruster publishers
    for (const auto& thruster_name : thruster_names_)
    {
        thruster_pubs_.push_back(
            this->create_publisher<std_msgs::msg::Float64>("cmd_" + thruster_name, 10));
    }
    
    RCLCPP_INFO(this->get_logger(), "Custom Thruster Manager initialized with custom TAM");
}

void CustomThrusterManager::declareParameters()
{
    this->declare_parameter("min_thrust", -40.0);
    this->declare_parameter("max_thrust", 40.0);
    this->declare_parameter("deadzone", 1.0);
    this->declare_parameter("thruster_names", std::vector<std::string>{"thruster1", "thruster2", "thruster3", "thruster4", "thruster5", "thruster6"});
}

void CustomThrusterManager::loadParameters()
{
    min_thrust_ = this->get_parameter("min_thrust").as_double();
    max_thrust_ = this->get_parameter("max_thrust").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    thruster_names_ = this->get_parameter("thruster_names").as_string_array();
}

void CustomThrusterManager::setupCustomTAM()
{
    // Initialize 6x6 TAM matrix (6 DOF x 6 thrusters)
    custom_tam_ = Eigen::MatrixXd::Zero(6, 6);
    
    // Based on actual thruster positions from URDF:
    // Thruster 1: (0.0549, 0.0, 0.0715) with rotation (0, 0, -π/2) - Side thruster pointing left
    // Thruster 2: (0.1265, 0.0747, 0.0) with rotation (0, 0, 0) - Forward thruster pointing forward  
    // Thruster 3: (0.1265, -0.0747, 0.0) with rotation (0, 0, 0) - Forward thruster pointing forward
    // Thruster 4: (0.1815, 0.0, 0.0) with rotation (0, -π/2, 0) - Vertical thruster pointing down
    // Thruster 5: (-0.1000, 0.0684, 0.0) with rotation (0, -π/2, 0) - Vertical thruster pointing down
    // Thruster 6: (-0.1000, -0.0684, 0.0) with rotation (0, -π/2, 0) - Vertical thruster pointing down
    
    // Order: [surge, sway, heave, roll, pitch, yaw]
    
    // Thruster 1: Side thruster - primarily sway, some yaw
    custom_tam_(1, 0) = -1.0;  // sway (negative because pointing left)
    custom_tam_(5, 0) = -0.0549;  // yaw (moment arm)
    
    // Thruster 2: Forward thruster - primarily surge, some yaw
    custom_tam_(0, 1) = 1.0;  // surge (forward)
    custom_tam_(5, 1) = 0.0747;  // yaw (positive moment arm)
    
    // Thruster 3: Forward thruster - primarily surge, some yaw
    custom_tam_(0, 2) = 1.0;  // surge (forward)
    custom_tam_(5, 2) = -0.0747;  // yaw (negative moment arm)
    
    // Thruster 4: Vertical thruster - primarily heave, some pitch
    custom_tam_(2, 3) = -1.0;  // heave (negative because pointing down)
    custom_tam_(4, 3) = -0.1815;  // pitch (negative moment arm)
    
    // Thruster 5: Vertical thruster - primarily heave, some pitch and roll
    custom_tam_(2, 4) = -1.0;  // heave (negative because pointing down)
    custom_tam_(4, 4) = -0.0684;  // pitch (negative moment arm)
    custom_tam_(3, 4) = 0.0684;  // roll (positive moment arm)
    
    // Thruster 6: Vertical thruster - primarily heave, some pitch and roll
    custom_tam_(2, 5) = -1.0;  // heave (negative because pointing down)
    custom_tam_(4, 5) = 0.0684;  // pitch (positive moment arm)
    custom_tam_(3, 5) = -0.0684;  // roll (negative moment arm)
    
    RCLCPP_INFO(this->get_logger(), "Custom TAM configured with actual thruster geometry:");
    RCLCPP_INFO(this->get_logger(), "Thruster 1: Side thruster (sway + yaw)");
    RCLCPP_INFO(this->get_logger(), "Thruster 2,3: Forward thrusters (surge + yaw)");
    RCLCPP_INFO(this->get_logger(), "Thruster 4,5,6: Vertical thrusters (heave + pitch + roll)");
}

void CustomThrusterManager::wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    // Convert wrench message to Eigen vector
    Eigen::VectorXd wrench(6);
    wrench << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
              msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    
    // Solve thruster allocation
    Eigen::VectorXd thrust = solveThrusterAllocation(wrench);
    
    // Publish thruster commands
    publishThrusterCommands(thrust);
}

Eigen::VectorXd CustomThrusterManager::solveThrusterAllocation(const Eigen::VectorXd& wrench)
{
    // Use pseudo-inverse to solve TAM * thrust = wrench
    Eigen::MatrixXd tam_inv = custom_tam_.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd thrust = tam_inv * wrench;
    
    // Scale thrust to respect limits
    scaleThrust(thrust);
    
    return thrust;
}

void CustomThrusterManager::scaleThrust(Eigen::VectorXd& thrust)
{
    double scale = 1.0;
    
    // Find the maximum scaling factor needed
    for (int i = 0; i < thrust.size(); ++i)
    {
        if (thrust(i) > 0)
        {
            scale = std::max(scale, thrust(i) / max_thrust_);
        }
        else if (thrust(i) < 0)
        {
            scale = std::max(scale, -thrust(i) / (-min_thrust_));
        }
    }
    
    // Apply scaling if needed
    if (scale > 1.0)
    {
        thrust /= scale;
    }
    
    // Apply deadzone with reduced threshold for smoother control
    for (int i = 0; i < thrust.size(); ++i)
    {
        if (std::abs(thrust(i)) < deadzone_ && thrust(i) != 0)
        {
            thrust(i) = (thrust(i) > 0) ? deadzone_ : -deadzone_;
        }
        
        // Additional smoothing: limit rapid changes
        if (std::abs(thrust(i)) > max_thrust_ * 0.8)
        {
            thrust(i) = (thrust(i) > 0) ? max_thrust_ * 0.8 : -max_thrust_ * 0.8;
        }
    }
}

void CustomThrusterManager::publishThrusterCommands(const Eigen::VectorXd& thrust)
{
    // Publish as JointState
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = thruster_names_;
    joint_state_msg->effort.resize(thrust.size());
    
    for (int i = 0; i < thrust.size(); ++i)
    {
        joint_state_msg->effort[i] = thrust(i);
    }
    
    joint_state_pub_->publish(*joint_state_msg);
    
    // Publish individual thruster commands
    for (Eigen::Index i = 0; i < thrust.size() && i < static_cast<Eigen::Index>(thruster_pubs_.size()); ++i)
    {
        auto thruster_msg = std::make_unique<std_msgs::msg::Float64>();
        thruster_msg->data = thrust(i);
        thruster_pubs_[i]->publish(*thruster_msg);
    }
}

} // namespace custom_thruster_manager
