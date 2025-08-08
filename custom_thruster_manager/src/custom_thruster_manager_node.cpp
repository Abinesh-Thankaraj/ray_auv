#include "custom_thruster_manager/custom_thruster_manager.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<custom_thruster_manager::CustomThrusterManager>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
