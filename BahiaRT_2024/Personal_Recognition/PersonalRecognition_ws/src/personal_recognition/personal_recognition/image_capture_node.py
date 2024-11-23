#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Importando a classe Image
from cv_bridge import CvBridge
import os
import cv2
import time

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__("image_capture_node")
        self.camera = cv2.VideoCapture(0)

        if not self.camera.isOpened():
            self.get_logger().error("Failed to open camera.")
            self.destroy_node()
            return

        # Publisher para publicar imagens no tópico /camera/image_raw
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.create_subscription(String, 'person_name', self.capture_callback, 10)
        self.create_subscription(String, 'next_capture_step', self.next_capture_step_callback, 10)

        self.ok_publisher = self.create_publisher(String, 'capture_status', 10)

        self.get_logger().info("Image capture Node has started")
        self.bridge = CvBridge()  # Inicializa o CvBridge para conversão de imagem
        self.reset_capture_state()

    def reset_capture_state(self):
        self.current_capture = 0
        self.max_captures = 3
        self.path = ""
        self.is_capturing = False
        self.global_capture_count = 0
        self.timer = None
        self.capture_sequence_count = 0  # Counter for sequences
        self.max_sequences = 3  # Maximum number of capture sequences
        self.can_capture = True  # Flag to control capturing state
        self.ok_sent = False  # Flag to indicate if "ok" has been sent

    def capture_callback(self, msg):
        self.get_logger().info(f"Received person name: {msg.data}")
        self.name = msg.data
        self.path = os.path.join(os.getcwd(), "images", self.name)
        os.makedirs(self.path, exist_ok=True)
        self.get_logger().info(f"Ready to capture images for: {self.name}")

    def capture_images(self):
        if self.is_capturing and self.current_capture < self.max_captures:
            self.capture_image()

        elif self.current_capture >= self.max_captures:
            self.get_logger().info("All captures done for this sequence.")

            if not self.ok_sent:
                self.ok_publisher.publish(String(data="ok"))
                self.get_logger().info("All captures done. Sending 'ok' and waiting for the next step.")
                self.ok_sent = True  # indicate "ok" sent

            self.is_capturing = False  # Stop capturing
            self.capture_sequence_count += 1 

            # Check if maximum capture sequences reached
            if self.capture_sequence_count >= self.max_sequences:
                self.get_logger().info("Maximum capture sequences reached. Shutting down...")
                self.cleanup()  # Call cleanup to release the camera
                self.can_capture = False 

            else:
                self.get_logger().info("Ready for the next sequence.")
                self.current_capture = 0  # Reset for the next sequence
                self.ok_sent = False  # Reset flag for the next sequence

    def next_capture_step_callback(self, msg):
        if msg.data == 'continue':
            if not self.can_capture:
                self.get_logger().info("Maximum capture sequences reached. No further captures allowed.")
                return

            if not self.path:
                self.get_logger().error("Path not set. Unable to capture images.")
                return

            if self.is_capturing:
                self.get_logger().info("Already capturing images. Please wait.")
                return

            self.is_capturing = True
            self.current_capture = 0  # Reset for new capture sequence
            self.global_capture_count = 0  # Reset global capture count
            self.get_logger().info("Starting image capture process...")
            self.capture_images()  # Start capturing images

    def capture_image(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error("Failed to capture image from the camera.")
            return

        # Salva a imagem e publica no tópico
        image_path = os.path.join(self.path, f'{self.name}_{self.global_capture_count}_{int(time.time())}.png')
        success = cv2.imwrite(image_path, frame)

        if success:
            self.get_logger().info(f"Image saved as {image_path}")
            self.global_capture_count += 1
            self.current_capture += 1
            self.get_logger().info(f"Captured {self.current_capture} images so far.")

            # Publica a imagem no tópico /camera/image_raw
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_publisher.publish(msg)
            self.get_logger().info("Published image to /camera/image_raw")

            if self.current_capture < self.max_captures:
                self.get_logger().info("Waiting before next capture...")

                if self.timer:
                    self.timer.cancel()
                self.timer = self.create_timer(3, self.capture_images)  # Set timer for next capture

        else:
            self.get_logger().error(f"Failed to save image at {image_path}")

    def cleanup(self):
        self.camera.release()  # Release the camera
        self.get_logger().info("Camera released.")

def main(args=None):
    rclpy.init(args=args)

    image_capture_node = ImageCaptureNode()

    try:
        rclpy.spin(image_capture_node)
    except KeyboardInterrupt:
        image_capture_node.get_logger().info("Node interrupted. Shutting down...")
    finally:
        image_capture_node.cleanup()  # Ensure camera is released on shutdown
        image_capture_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
