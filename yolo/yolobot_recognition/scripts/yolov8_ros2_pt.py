#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # Load YOLO model - use the local model file
        model_path = os.path.join(os.path.dirname(__file__), 'yolov8n.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model from: {model_path}')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/ray/image',  # Ray AUV camera topic
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        
        # Display window for real-time viewing
        self.create_timer(0.1, self.display_image)
        self.current_image = None
        
        # Detection tracking for continuous and stable detection
        self.last_detection_time = self.get_clock().now()
        self.detection_history = []  # Store recent detections
        self.max_history = 5  # Keep last 5 frames
        
        # Persistent detection tracking
        self.last_valid_boxes = []  # Store last detected boxes
        self.box_persistence_frames = 10  # Keep boxes for N frames
        self.frames_since_detection = 0
        
        self.get_logger().info('Camera subscriber initialized - subscribing to /ray/image')
        self.get_logger().info('STINGRAY IN MISSION: ULTRA-sensitive persistent detection (3% confidence)')
        self.get_logger().info('Enhanced image processing + persistent tracking (10 frames)')
        self.get_logger().info('Bounding boxes will stay visible for continuous tracking')
        self.get_logger().info('Display window will open - Press Q to quit')

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Apply image preprocessing for better detection
        # Increase brightness and contrast to help YOLO see better
        img_enhanced = cv2.convertScaleAbs(img, alpha=1.2, beta=20)
        
        # Run YOLO inference with ULTRA-LOW confidence threshold for continuous detection
        # Extremely aggressive settings to detect ANY person or body part continuously
        results = self.model(
            img_enhanced,   # Use enhanced image
            conf=0.03,      # ULTRA-LOW 3% confidence for maximum sensitivity
            iou=0.2,        # Very low IOU to catch all possible detections
            classes=[0],    # Person class only
            agnostic_nms=True,  # Better detection of partial views
            max_det=20,     # Allow up to 20 detections
            half=False,     # Use full precision for better accuracy
            verbose=False,  # Reduce console spam
            retina_masks=False  # Faster processing
        )

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        person_count = 0
        current_detections = []
        current_boxes = []
        
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    # Only process person detections (class 0)
                    if int(box.cls) == 0:
                        person_count += 1
                        confidence = float(box.conf)
                        
                        self.inference_result = InferenceResult()
                        b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                        c = box.cls
                        self.inference_result.class_name = self.model.names[int(c)]
                        self.inference_result.top = int(b[0])
                        self.inference_result.left = int(b[1])
                        self.inference_result.bottom = int(b[2])
                        self.inference_result.right = int(b[3])
                        self.yolov8_inference.yolov8_inference.append(self.inference_result)
                        
                        # Store detection for history and persistence
                        box_data = {
                            'bbox': (int(b[0]), int(b[1]), int(b[2]), int(b[3])),
                            'conf': confidence,
                            'frames_alive': 0
                        }
                        current_detections.append(box_data)
                        current_boxes.append(box_data)
                        
                        # Log detection with more info
                        self.get_logger().info(f'✓ Person detected: conf={confidence:.3f}, bbox=({int(b[0])}, {int(b[1])}, {int(b[2])}, {int(b[3])})')
        
        # Update persistent boxes
        if len(current_boxes) > 0:
            self.last_valid_boxes = current_boxes
            self.frames_since_detection = 0
        else:
            self.frames_since_detection += 1
        
        # Update detection history
        self.detection_history.append(person_count)
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
        
        # Calculate average detections over recent frames
        avg_detections = sum(self.detection_history) / len(self.detection_history) if self.detection_history else 0

        # Use enhanced image for display
        annotated_frame = results[0].plot()
        
        # Draw persistent boxes if no current detection but within persistence window
        if person_count == 0 and self.frames_since_detection < self.box_persistence_frames and len(self.last_valid_boxes) > 0:
            for box_data in self.last_valid_boxes:
                bbox = box_data['bbox']
                conf = box_data['conf']
                # Draw persistent box with different color (yellow/orange)
                cv2.rectangle(annotated_frame, 
                            (bbox[0], bbox[1]), (bbox[2], bbox[3]), 
                            (0, 165, 255), 2)  # Orange color for persistent boxes
                # Add label
                label = f'Person {conf:.2f} (tracking)'
                cv2.putText(annotated_frame, label, 
                          (bbox[0], bbox[1] - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        
        # Add minimalistic mission text at top-left corner
        cv2.putText(annotated_frame, 'Stingray in Mission', 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Add person count with detection status
        if person_count > 0:
            color = (0, 255, 0)  # Green - active detection
            status = "DETECTING"
        elif self.frames_since_detection < self.box_persistence_frames and len(self.last_valid_boxes) > 0:
            color = (0, 165, 255)  # Orange - tracking
            status = "TRACKING"
        else:
            color = (0, 0, 255)  # Red - no detection
            status = "SEARCHING"
        
        cv2.putText(annotated_frame, f'{status} | Persons: {person_count} | Avg: {avg_detections:.1f}', 
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add confidence threshold info
        cv2.putText(annotated_frame, 'Conf: 3%', 
                   (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Store current image for display window
        self.current_image = annotated_frame
        
        # Fix image encoding for ROS publishing
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        img_msg.header = data.header

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    def display_image(self):
        """Display the current image in a window"""
        if self.current_image is not None:
            cv2.imshow('Ray AUV - Person Detection', self.current_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed. Shutting down...')
                rclpy.shutdown()

def main():
    rclpy.init(args=None)
    
    try:
        camera_subscriber = Camera_subscriber()
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
