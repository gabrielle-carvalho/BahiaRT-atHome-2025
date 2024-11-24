#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image 
import cv2
from ultralytics import YOLO
import os
from cv_bridge import CvBridge
from datetime import datetime
import time

class ObjectRecognition(Node):
    def __init__(self):
        super().__init__('object_recognition_node')

        self.create_subscription(String, 'publishing_recognized', self.person_callback, 10)
        self.detected_publisher = self.create_publisher(String, 'detected_objects', 10)
        self.get_logger().info("Object detection started.")

        self.create_subscription(Image, '/camera/image_raw', self.recognition_callback, 10) 

        self.br = CvBridge()

        self.classNames = ['Cleaning supplies- Cloth', 'Cleaning supplies- Soap', 'Cleaning supplies- SteelWool', 
                           'Drinks- Banana', 'Drinks- Coke', 'Drinks- Fanta', 'Drinks- Kuat', 'Drinks- Papaya', 
                           'Drinks- Pithula', 'Fruits- Apple', 'Fruits- Kiwi', 'Fruits- Lemon', 'Fruits- Sicilian_Lemon', 
                           'Pantry Items- Gelatin', 'Pantry Items- ToothPick', 'Pantry items- Cappuccino', 
                           'Pantry items- MilkCream', 'Pantry items- TomatoSauce', 'Snacks- Wafer']

        model_path = "/home/bill7/BahiaRT_athome/ros2_ws/src/qualification_submission/qualification_submission/yolo/best.pt"
        self.model = YOLO(model_path)

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found: {model_path}")
        else:
            self.get_logger().info(f"YOLO model loaded successfully!")

        self.recognition_enabled = False
        self.camera = None

        self.save_dir = '/home/bill7/BahiaRT_athome/ros2_ws/src/qualification_submission/qualification_submission/predict_object'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.max_camera_retries = 5
        self.camera_retry_interval = 2

    def open_camera(self):
        if self.camera is not None:
            return True  

        self.cap = cv2.VideoCapture(0)
        retries = 0
        while retries < self.max_camera_retries:
            if self.cap.isOpened():
                self.camera = self.cap  
                self.get_logger().info("Camera opened successfully.")
                return True
            else:
                retries += 1
                self.get_logger().error(f"Failed to open camera. Attempt {retries}/{self.max_camera_retries}. Retrying in {self.camera_retry_interval} seconds...")
                time.sleep(self.camera_retry_interval)

        self.get_logger().error("Exceeded maximum retries. Camera could not be opened.")
        return False

    def person_callback(self, msg):
        self.get_logger().info("Message received on /turned_around.")
        self.recognition_enabled = True
        self.get_logger().info("Recognition enabled.")

        if not self.open_camera():
            return

        self.create_timer(0.1, self.capture_and_process_frame)

    def recognition_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        self.process_image(frame)

    def capture_and_process_frame(self):
        if not hasattr(self, 'cap') or self.cap is None or not self.cap.isOpened():
            self.get_logger().error("Camera is not available or not opened.")
            if self.open_camera():
                return
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image from camera.")
            return

        self.process_image(frame)

    def process_image(self, frame):
        if not self.recognition_enabled:
            return

        frame_resized = cv2.resize(frame, (640, 640))
        results = self.model(frame_resized, conf=0.6)

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = int(box.conf[0] * 100)
            cls = int(box.cls[0])

            object_name = self.classNames[cls]
            self.get_logger().info(f"Object detected: {object_name}, Confidence: {confidence}%")
            self.detected_publisher.publish(String(data=object_name))

            cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{object_name}: {confidence}%"
            cv2.putText(frame_resized, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            self.save_detected_image(frame_resized, object_name, confidence)

        cv2.imshow("Detected Objects", frame_resized)
        cv2.waitKey(1)

    def save_detected_image(self, frame, object_name, confidence):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_name = f"{object_name}_{confidence}percent_{timestamp}.jpg"
        file_path = os.path.join(self.save_dir, file_name)

        cv2.imwrite(file_path, frame)
        self.get_logger().info(f"Saved detected object image as {file_name}")

    def cleanup(self):
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    object_recognition_node = ObjectRecognition()

    try:
        rclpy.spin(object_recognition_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_recognition_node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
