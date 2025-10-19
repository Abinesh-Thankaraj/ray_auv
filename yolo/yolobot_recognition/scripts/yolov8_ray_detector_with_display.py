#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class RayYOLODetectorWithDisplay(Node):
    def __init__(self):
        super().__init__('ray_yolo_detector_with_display')
        
        # Load YOLO model - use the local model file
        model_path = os.path.join(os.path.dirname(__file__), 'yolov8n.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model from: {model_path}')
        
        # Initialize inference message
        self.yolov8_inference = Yolov8Inference()
        
        # Subscribe to Ray AUV camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/ray/image',  # Ray AUV camera topic
            self.camera_callback,
            10)
        
        # Publishers for inference results and annotated image
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        
        # Display window for real-time viewing
        self.create_timer(0.1, self.display_image)
        self.current_image = None
        self.detection_count = 0
        
        self.get_logger().info('Ray YOLO Detector with Display initialized - subscribing to /ray/image')
        self.get_logger().info('Press Ctrl+C to stop')

    def camera_callback(self, data):
        try:
            # Convert ROS image to OpenCV format
            img = bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Run YOLO inference
            results = self.model(img, conf=0.5, classes=[0])  # Only detect person class (class 0)
            
            # Prepare inference message
            self.yolov8_inference.header.frame_id = "ray/camera_optical"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
            self.yolov8_inference.yolov8_inference.clear()
            
            # Process detections
            person_count = 0
            for r in results:
                boxes = r.boxes
                if boxes is not None:
                    for box in boxes:
                        # Only process person detections (class 0)
                        if int(box.cls) == 0:  # Person class
                            person_count += 1
                            self.inference_result = InferenceResult()
                            b = box.xyxy[0].to('cpu').detach().numpy().copy()
                            c = box.cls
                            
                            self.inference_result.class_name = self.model.names[int(c)]
                            self.inference_result.top = int(b[0])
                            self.inference_result.left = int(b[1])
                            self.inference_result.bottom = int(b[2])
                            self.inference_result.right = int(b[3])
                            
                            self.yolov8_inference.yolov8_inference.append(self.inference_result)
                            
                            # Log detection
                            self.get_logger().info(f'Person detected #{self.detection_count + person_count}: '
                                                 f'bbox=({self.inference_result.top}, {self.inference_result.left}, '
                                                 f'{self.inference_result.bottom}, {self.inference_result.right})')
            
            # Create annotated frame
            annotated_frame = results[0].plot()
            
            # Add detection count and status to image
            cv2.putText(annotated_frame, f'Persons detected: {person_count}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Add frame info
            cv2.putText(annotated_frame, f'Ray AUV - Person Detection', 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add instructions
            cv2.putText(annotated_frame, f'Press Q to quit', 
                       (10, annotated_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Store current image for display
            self.current_image = annotated_frame
            self.detection_count += person_count
            
            # Publish annotated image
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            img_msg.header = data.header
            self.img_pub.publish(img_msg)
            
            # Publish inference results
            self.yolov8_pub.publish(self.yolov8_inference)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def display_image(self):
        """Display the current image in a single window"""
        if self.current_image is not None:
            cv2.imshow('Ray AUV - Person Detection', self.current_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                self.get_logger().info('Quit key pressed. Shutting down...')
                rclpy.shutdown()

def main():
    rclpy.init(args=None)
    
    try:
        detector = RayYOLODetectorWithDisplay()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
