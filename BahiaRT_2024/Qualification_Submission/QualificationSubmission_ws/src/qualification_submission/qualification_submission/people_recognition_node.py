#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import os
import cv2
import dlib
import numpy as np
from scipy.spatial import distance
from sensor_msgs.msg import Image

class PeopleRecognition(Node):
    def __init__(self):
        super().__init__("people_recognition_node")
        self.camera = None
        
        self.detector = dlib.get_frontal_face_detector()
        self.sp = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')
        self.facerec = dlib.face_recognition_model_v1('dlib_face_recognition_resnet_model_v1.dat')

        self.capture_sequence_count = 0
        self.max_sequences = 10

        self.create_subscription(Image, '/camera/image_raw', self.recognition_callback, 10)
        self.create_subscription(String, 'turned_around', self.turned_callback, 10)
        self.labels, self._descriptors = self.prepare_training_data("images")

        self.name_publisher = self.create_publisher(String, 'recognized_person', 10)
        self.count_publisher = self.create_publisher(String, 'count_people', 10)
        self.recognition_completed_publisher = self.create_publisher(String, '/publishing_recognized', 10)

        self.get_logger().info("Face Recognition Node has Started")
        self.bridge = CvBridge()
        self.reset_capture_state()

    def reset_capture_state(self):
        self.started = False
        self.recognition_enabled = False
        self.total_faces_detected = 0
        self.detection_count = {}
        self.recognition_count = 0
        self.can_capture = False

    def turned_callback(self, msg):
        self.get_logger().info(f"Received turned_around message: {msg.data}")
        self.reset_capture_state()
        self.recognition_enabled = True
        self.get_logger().info(f"Recognition enabled: {self.recognition_enabled}")
        
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().info("Opening camera...")
            self.open_camera()
            if self.camera is None or not self.camera.isOpened():
                self.get_logger().error("Failed to open camera.")
            else:
                self.get_logger().info("Camera opened successfully.")
        else:
            self.get_logger().info("Camera already open.")

    def open_camera(self):
        self.camera = cv2.VideoCapture(0)
        max_attempts = 5000
        attempts = 0
        while attempts < max_attempts:
            if self.camera.isOpened():
                return True
            else:
                self.get_logger().warning(f"Attempt {attempts + 1}: Failed to open camera.")
                self.camera.release()
                self.camera = cv2.VideoCapture(0)
                attempts += 1
        return False

    def prepare_training_data(self, data_folder_path):
        labels = []
        descriptors = []
        if not os.path.exists(data_folder_path):
            self.get_logger().error(f"Data folder {data_folder_path} does not exist.")
            return labels, descriptors

        for label in os.listdir(data_folder_path):
            person_dir = os.path.join(data_folder_path, label)
            if not os.path.isdir(person_dir):
                self.get_logger().warning(f"{person_dir} is not a directory.")
                continue

            for image_name in os.listdir(person_dir):
                image_path = os.path.join(person_dir, image_name)
                img = cv2.imread(image_path)
                if img is None:
                    self.get_logger().warning(f"Could not read image {image_path}. Skipping.")
                    continue

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                faces = self.detector(gray)

                if len(faces) == 0:
                    self.get_logger().warning(f"No faces detected in image {image_path}.")
                    continue

                for face in faces:
                    shape = self.sp(gray, face)
                    face_descriptor = self.facerec.compute_face_descriptor(img, shape)
                    descriptors.append(np.array(face_descriptor))
                    labels.append(label)

        self.get_logger().info(f"Total labels: {len(labels)}, Total descriptors: {len(descriptors)}")
        return labels, descriptors

    def recognition_callback(self, msg):
        self.get_logger().info(f"Recognition enabled: {self.recognition_enabled}")
        if not self.recognition_enabled:
            self.get_logger().info("Recognition is not enabled.")
            return

        if self.camera is None or not self.camera.isOpened():
            self.get_logger().error("Camera is not open. Skipping recognition callback.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info(f"Captured frame with shape: {frame.shape}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detector(gray)
        current_faces_detected = len(faces)

        self.get_logger().info(f"Detected {current_faces_detected} faces on screen.")

        if current_faces_detected == 0:
            return

        for face in faces:
            shape = self.sp(gray, face)
            face_descriptor = self.facerec.compute_face_descriptor(frame, shape)

            if face_descriptor is None:
                self.get_logger().warning("Face descriptor could not be computed.")
                continue  

            name = self.match_face(np.array(face_descriptor), self.labels, self._descriptors)

            if name not in self.detection_count:
                self.detection_count[name] = 0

            if name != "Unknown":
                self.detection_count[name] += 1
                self.get_logger().info(f"Recognized: {name}, Count: {self.detection_count[name]}")
                self.save_recognized_face(frame, name, face)

                if self.detection_count[name] == 12:
                    self.recognition_count += 1
                    self.name_publisher.publish(String(data=name))
                    self.get_logger().info(f"Published recognized name: {name}")
                    self.count_publisher.publish(String(data=str(self.total_faces_detected)))
                    self.get_logger().info(f"Recognition limit reached for {name}. Total recognitions: {self.recognition_count}")
                    self.recognition_enabled = False

                    self.publish_recognition_completed(name)
                    self.close_camera() 

                    self.detection_count[name] = 0
                    return
            else:
                self.detection_count[name] = 0
                self.save_recognized_face(frame, "Unknown", face)

            x, y, w, h = face.left(), face.top(), face.width(), face.height()
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, name if name != "Unknown" else "Unknown", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.total_faces_detected = current_faces_detected
        self.get_logger().info(f"Total faces detected on screen: {self.total_faces_detected}")

    def publish_recognition_completed(self, recognized_name):
        msg = String()
        msg.data = recognized_name 
        self.recognition_completed_publisher.publish(msg)
        self.get_logger().info(f"Published recognized name: {recognized_name}")
        
    def save_recognized_face(self, frame, name, face):
        if frame is None or not isinstance(frame, np.ndarray):
            self.get_logger().error("Invalid frame, cannot save.")
            return

        x, y, w, h = face.left(), face.top(), face.height(), face.width()
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        file_name = os.path.join('predict_person', f"{name}.jpg")
        count = 1
        while os.path.exists(file_name):
            file_name = os.path.join('predict_person', f"{name}_{count}.jpg")
            count += 1

        self.get_logger().info(f"Saving image to: {file_name}")

        if cv2.imwrite(file_name, frame):
            self.get_logger().info(f"Saved recognized face: {file_name}")
        else:
            self.get_logger().error(f"Failed to save image: {file_name}")

    def match_face(self, face_descriptor, labels, descriptors):
        if len(descriptors) == 0:
            self.get_logger().warning("No face descriptors available for matching.")
            return "Unknown"
        
        distances = [distance.euclidean(face_descriptor, descriptor) for descriptor in descriptors]
        min_distance = min(distances)

        if min_distance < 0.6:
            index = distances.index(min_distance)
            return labels[index]
        
        return "Unknown"

def main(args=None):
    rclpy.init(args=args)
    people_recognition_node = PeopleRecognition()
    rclpy.spin(people_recognition_node)
    people_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
