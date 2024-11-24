#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
from ultralytics import YOLO
import os
import time
#ele está salvando um predict, devido a condição de 15 em 15
import shutil
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from PIL import Image as PILImage
import matplotlib.pyplot as plt
from cv_bridge import CvBridge


class ObjectRecognition(Node):
    def __init__(self):
        super().__init__('object_recognition_node')
        

        self.camera = cv2.VideoCapture(1)

        if not self.camera.isOpened():
            self.get_logger().error("Falha ao abrir a câmera.")
            self.get_logger().error(f"Erro ao abrir a câmera: {cv2.VideoCapture(0).isOpened()}")
            self.destroy_node()
            return
        else:
            self.get_logger().info("Câmera aberta com sucesso.")

        self.detected_publisher = self.create_publisher(String, 'detected_objects', 10)
        self.get_logger().info("Publicador de objetos detectados iniciado.")
        
        self.br = CvBridge()

        self.predict_folder = "/home/bill7/BahiaRT_athome/ros2_ws/src/object_recognition/object_recognition/predict"
        
        if not os.path.exists(self.predict_folder):
            os.makedirs(self.predict_folder)

        model_path = "/home/bill7/BahiaRT_athome/ros2_ws/src/object_recognition/object_recognition/yolo/best.pt"
        self.classNames = ['Cleaning supplies- Cloth', 'Cleaning supplies- Soap', 'Cleaning supplies- SteelWool', 
                           'Drinks- Banana', 'Drinks- Coke', 'Drinks- Fanta', 'Drinks- Kuat', 'Drinks- Papaya', 
                           'Drinks- Pithula', 'Fruits- Apple', 'Fruits- Kiwi', 'Fruits- Lemon', 'Fruits- Sicilian_Lemon', 
                           'Pantry Items- Gelatin', 'Pantry Items- ToothPick', 'Pantry items- Cappuccino', 
                           'Pantry items- MilkCream', 'Pantry items- TomatoSauce', 'Snacks- Wafer']

        self.model = YOLO(model_path)
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo não encontrado: {model_path}")
        else:
            self.get_logger().info(f"Modelo YOLO carregado com sucesso!")

        self.detected_count = 0
        self.textos_para_pdf = []
        self.imagens_para_pdf = []

        self.max_detected_objects = 25
        self.detected_objects_count = 0
        self.frame_counter = 0
        self.max_frame_interval = 2

    def process_image(self, frame):
        self.frame_counter += 1
        
        if self.frame_counter % self.max_frame_interval == 0:
            frame_resized = cv2.resize(frame, (640, 640))

            results = self.model(frame_resized, conf=0.6)

            if len(results[0].boxes) > 0:
                self.get_logger().info(f"{len(results[0].boxes)} objetos detectados.")
            else:
                self.get_logger().info("Nenhum objeto detectado.")
                return  # Se não houver objetos detectados, simplesmente retorna e não faz mais nada

            detected_objects = []  

            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  
                confidence = int(box.conf[0] * 100)
                cls = int(box.cls[0])  

                object_name = self.classNames[cls]
                detected_objects.append((object_name, confidence, (x1, y1, x2, y2)))

                self.get_logger().info(f"Objeto detectado: {object_name}, Confiança: {confidence}%")

                label = f"{object_name}: {confidence}%"
                org = (x1, y1 - 10)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(frame_resized, label, org, font, fontScale, color, thickness)
                cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Publicando o nome do objeto detectado
                self.detected_publisher.publish(String(data=object_name))
                self.get_logger().info("Publicando objeto reconhecido")

                self.detected_objects_count += 1
                if self.detected_objects_count >= self.max_detected_objects:
                    self.get_logger().info(f"Limite de {self.max_detected_objects} objetos atingido. Parando o reconhecimento.")
                    rclpy.shutdown()
                    #break
            
            
            # Se houver objetos detectados, salva a imagem e cria o PDF
            if self.detected_objects_count:
                self.save_detected_image(frame_resized, detected_objects)

            cv2.imshow("Detected Objects", frame_resized)
            cv2.waitKey(1)

    def save_detected_image(self, frame, detected_objects):
        timestamp = int(time.time())
        image_filename = f"{self.predict_folder}/detected_image_{timestamp}.jpg"
        cv2.imwrite(image_filename, frame)
        self.get_logger().info(f"Imagem com deteções salva como {image_filename}")

        self.create_pdf(detected_objects, image_filename)

        self.detected_count += 1

    def create_pdf(self, detected_objects, image_path):
        self.textos_para_pdf.append("\n".join([f"Objeto: {obj_name}, Confiança: {confidence}%" 
                                              for obj_name, confidence, _ in detected_objects]))
        self.imagens_para_pdf.append(image_path)

        if len(self.textos_para_pdf) >= 1 and len(self.imagens_para_pdf) >= 1:
            self.criar_pdf_com_texto_e_imagem()

    def criar_pdf_com_texto_e_imagem(self, nome_arquivo="/media/bill7/backup/Recognized_objects.pdf"):
        min_len = min(len(self.textos_para_pdf), len(self.imagens_para_pdf))
        if min_len == 0:
            self.get_logger().error("Nenhuma detecção válida foi feita para gerar o PDF.")
            return

        c = canvas.Canvas(nome_arquivo, pagesize=A4)
        largura, altura = A4

        for i in range(min_len):
            texto = self.textos_para_pdf[i]
            imagem = self.imagens_para_pdf[i]

            c.setFont("Helvetica", 12)
            c.drawString(50, altura - 50, texto)

            caminho_imagem = os.path.join("predict", imagem)

            try:
                with PILImage.open(caminho_imagem) as img:
                    largura_imagem, altura_imagem = img.size

                    max_largura = largura - 100
                    max_altura = altura - 150
                    proporcao = min(max_largura / largura_imagem, max_altura / altura_imagem)
                    nova_largura = int(largura_imagem * proporcao)
                    nova_altura = int(altura_imagem * proporcao)
                    img = img.resize((nova_largura, nova_altura))

                    temp_image_path = f"temp_image_{i}.jpg"
                    img.save(temp_image_path)
                    c.drawImage(temp_image_path, 50, altura - nova_altura - 100, width=nova_largura, height=nova_altura)

                    os.remove(temp_image_path)

            except Exception as e:
                self.get_logger().error(f"Erro ao abrir a imagem {caminho_imagem}: {e}")
                continue

            c.showPage()

        c.save()
        self.get_logger().info(f"PDF gerado: {nome_arquivo}")

    def run(self):
        while rclpy.ok():
            ret, frame = self.camera.read()
            if ret:
                self.process_image(frame)
            else:
                self.get_logger().error("Falha ao capturar imagem da câmera.")
                break

    def destroy(self):
        self.camera.release()
        cv2.destroyAllWindows()  


def main(args=None):
    rclpy.init(args=args)
    object_recognition_node = ObjectRecognition()
    object_recognition_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
