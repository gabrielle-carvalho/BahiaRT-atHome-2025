#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import playsound
import os

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        
        # Subscribers
        self.subscription_object_detected = self.create_subscription(String, '/detected_objects', self.capture_status_callback, 10)
        
        self.subscription_object_detected  # Para evitar o erro de variável não utilizada
            
    def speak(self, text):  
        # Certifique-se de que o diretório "audios" existe
        if not os.path.exists("audios"):
            os.makedirs("audios")

        tts = gTTS(text=text, lang="en")  # Mantenha o idioma como 'en' ou mude para 'pt' para português
        audio_file = "audios/tts_output.mp3"
        tts.save(audio_file)

        playsound(audio_file)

        os.remove(audio_file)

    def speak_object_name(self, msg):
        if msg.data != 'Unknown':
            self.speak(msg)
    
    def capture_status_callback(self, msg):
        self.speak_object_name(msg)
        
def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceNode()
    rclpy.spin(voice_node)
    voice_node.destroy_node()
    rclpy.shutdown()
