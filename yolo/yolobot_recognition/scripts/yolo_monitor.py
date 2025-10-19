#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import Yolov8Inference
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class YOLOMonitor(Node):
    def __init__(self):
        super().__init__('yolo_monitor')
        
        self.bridge = CvBridge()
        self.detection_count = 0
        
        # Subscribe to YOLO inference results
        self.inference_sub = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.inference_callback,
            10
        )
        
        # Subscribe to annotated image
        self.image_sub = self.create_subscription(
            Image,
            '/inference_result',
            self.image_callback,
            10
        )
        
        # Timer for display updates
        self.create_timer(0.1, self.display_image)
        self.current_image = None
        
        self.get_logger().info('YOLO Monitor started - monitoring person detections')
        self.get_logger().info('Press Ctrl+C to stop')

    def inference_callback(self, msg):
        """Process YOLO inference results"""
        if msg.yolov8_inference:
            self.detection_count += 1
            for detection in msg.yolov8_inference:
                self.get_logger().info(f'Detection #{self.detection_count}: {detection.class_name} '
                                     f'at bbox=({detection.top}, {detection.left}, '
                                     f'{detection.bottom}, {detection.right})')
        else:
            # No detections in this frame
            pass

    def image_callback(self, msg):
        """Store the current annotated image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def display_image(self):
        """Display the current image in a window"""
        if self.current_image is not None:
            cv2.imshow('Ray AUV - YOLO Person Detection', self.current_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed. Shutting down...')
                rclpy.shutdown()

def main():
    rclpy.init()
    
    try:
        monitor = YOLOMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
