#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
import math
import random

class RayPressureSensor(Node):
    def __init__(self):
        super().__init__('ray_pressure_sensor')
        
        # Subscribe to odometry to get robot position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ray/odom',
            self.odom_callback,
            10
        )
        
        # Publishers for pressure sensor data
        self.pressure_pub = self.create_publisher(FluidPressure, '/ray/pressure', 10)
        self.depth_pub = self.create_publisher(Float64, '/ray/depth_meters', 10)
        self.bar_pub = self.create_publisher(Float64, '/ray/pressure_bar', 10)
        
        # Physical constants for underwater pressure calculation
        self.atmospheric_pressure_pa = 101325.0  # Pa (1 atm)
        self.water_density = 1025.0  # kg/m³ (seawater)
        self.gravity = 9.80665  # m/s²
        self.pressure_per_meter = self.water_density * self.gravity  # Pa/m
        
        # Sensor noise parameters
        self.noise_std = 100.0  # Pa standard deviation
        
        # Current robot state
        self.current_depth = 0.0
        
        # Silent startup for clean world launches
        # self.get_logger().info('🌊 RAY Pressure Sensor started')
    
    def odom_callback(self, msg):
        """Process odometry data and calculate pressure sensor readings"""
        # Extract robot depth from Z position
        z_position = msg.pose.pose.position.z
        self.current_depth = -z_position  # Convert to positive depth
        
        # Calculate expected pressure based on depth
        expected_pressure = self.atmospheric_pressure_pa + (self.water_density * self.gravity * self.current_depth)
        
        # Add realistic sensor noise
        noise = random.gauss(0, self.noise_std)
        measured_pressure_pa = expected_pressure + noise
        
        # Calculate depth from measured pressure (like real pressure sensor)
        calculated_depth = (measured_pressure_pa - self.atmospheric_pressure_pa) / self.pressure_per_meter
        
        # Publish FluidPressure message
        pressure_msg = FluidPressure()
        pressure_msg.fluid_pressure = measured_pressure_pa
        pressure_msg.variance = self.noise_std * self.noise_std  # Variance = std²
        self.pressure_pub.publish(pressure_msg)
        
        # Publish calculated depth (negative for underwater depth)
        depth_msg = Float64()
        depth_msg.data = -calculated_depth  # Negative depth for underwater
        self.depth_pub.publish(depth_msg)
        
        # Publish pressure in bar
        bar_msg = Float64()
        bar_msg.data = measured_pressure_pa / 100000.0  # Convert Pa to bar
        self.bar_pub.publish(bar_msg)
        
        # Log every 50th reading to avoid spam
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        # No periodic logging to keep terminal clean
        # Uncomment the lines below if you want to see logs when running separately
        # if self.log_counter % 1000 == 0:
        #     self.get_logger().info(
        #         f'🌊 Pressure: {measured_pressure_pa/1000:.1f}kPa, '
        #         f'Depth: {-calculated_depth:.3f}m, '
        #         f'Bar: {measured_pressure_pa/100000:.3f}bar'
        #     )

def main(args=None):
    rclpy.init(args=args)
    node = RayPressureSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
