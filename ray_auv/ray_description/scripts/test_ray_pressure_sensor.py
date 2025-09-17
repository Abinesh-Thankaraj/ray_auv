#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64
import time

class RayPressureSensorTester(Node):
    def __init__(self):
        super().__init__('ray_pressure_sensor_tester')
        
        # Subscribe to pressure sensor data
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            '/ray/pressure',
            self.pressure_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Float64,
            '/ray/depth_meters',
            self.depth_callback,
            10
        )
        
        self.bar_sub = self.create_subscription(
            Float64,
            '/ray/pressure_bar',
            self.bar_callback,
            10
        )
        
        self.current_pressure = None
        self.current_depth = None
        self.current_bar = None
        self.count = 0
        
        self.get_logger().info('🧪 RAY Pressure Sensor Tester started')
        self.get_logger().info('📊 Monitoring sensor_msgs/FluidPressure messages...')
    
    def pressure_callback(self, msg):
        self.current_pressure = msg.fluid_pressure
        self.print_status()
    
    def depth_callback(self, msg):
        self.current_depth = msg.data
    
    def bar_callback(self, msg):
        self.current_bar = msg.data
    
    def print_status(self):
        self.count += 1
        if self.count % 10 == 0:  # Print every 10th reading
            if all(x is not None for x in [self.current_pressure, self.current_depth, self.current_bar]):
                self.get_logger().info(
                    f'🌊 Reading #{self.count}: '
                    f'Pressure={self.current_pressure/1000:.1f}kPa, '
                    f'Depth={self.current_depth:.3f}m, '
                    f'Bar={self.current_bar:.3f}bar'
                )

def main(args=None):
    rclpy.init(args=args)
    node = RayPressureSensorTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
