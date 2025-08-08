#ifndef CUSTOM_THRUSTER_MANAGER_H
#define CUSTOM_THRUSTER_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace custom_thruster_manager
{

class CustomThrusterManager : public rclcpp::Node
{
public:
    explicit CustomThrusterManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ROS 2 components
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> thruster_pubs_;
    
    // TAM configuration for custom thruster allocation
    Eigen::MatrixXd custom_tam_;
    
    // Thruster limits
    double min_thrust_;
    double max_thrust_;
    double deadzone_;
    
    // Thruster names
    std::vector<std::string> thruster_names_;
    
    // Callback functions
    void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    
    // Custom TAM computation based on user requirements
    void setupCustomTAM();
    
    // Thruster allocation solver
    Eigen::VectorXd solveThrusterAllocation(const Eigen::VectorXd& wrench);
    
    // Utility functions
    void scaleThrust(Eigen::VectorXd& thrust);
    void publishThrusterCommands(const Eigen::VectorXd& thrust);
    
    // Parameters
    void declareParameters();
    void loadParameters();
};

} // namespace custom_thruster_manager

#endif // CUSTOM_THRUSTER_MANAGER_H
