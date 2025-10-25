#include "ray_thruster_manager/ray_thruster_manager.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ray_thruster_manager::RayThrusterManager>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
