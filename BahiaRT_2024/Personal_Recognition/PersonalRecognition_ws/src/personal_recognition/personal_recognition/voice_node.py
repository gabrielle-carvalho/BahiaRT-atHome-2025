#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
from gtts import gTTS
import os
from playsound import playsound
import noisereduce as nr
import numpy as np
import time
import threading

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # Publishers
        self.publisher_name = self.create_publisher(String, 'person_name', 10)
        self.publisher_training_status = self.create_publisher(String, 'training_finished', 10)
        self.publisher_next_step = self.create_publisher(String, 'next_capture_step', 10)
        
        # Subscribers
        self.subscription_capture = self.create_subscription(String, '/capture_status', self.capture_status_callback, 10)
        
        self.subscription_recognized = self.create_subscription(String, '/recognized_person', self.recognized_callback, 10)
        self.subscription_capture_finished = self.create_subscription(String, '/capture_finished', self.capture_finished_callback, 10)

        
        self.subscription_name = self.create_subscription(String, 'person_name', self.person_name_callback, 10)


        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info("Voice Node has started")

        self.keyword_detected = False  # Flag para garantir que a palavra-chave só seja detectada uma vez
        self.current_step_index = 0
        self.capture_status_ok = True  # Flag para aguardar o "OK"
        
        # Define capture steps
        self.capture_steps = [
            "aproximar_da_camera",
            "vira_rosto_direita",
            "vira_rosto_esquerda",
        ]
        
        # Audios para interacao
        # current_directory = os.path.dirname(os.path.abspath(__file__))
        self.audio_file_path_waiting = os.path.join("audios/microphoneOn.mp3")
        self.audio_file_path_detected = os.path.join("audios/keywordDetected.mp3")
        self.audio_file_path_dont_understand = os.path.join("audios/dontUnderstand.mp3")
        self.audio_file_path_error = os.path.join("audios/erro.mp3")
        self.audio_file_path_training_finished = os.path.join("audios/treinamentoFinalizado.mp3")
        self.audio_file_path_whats_your_name = os.path.join("audios/whatsYourName.mp3")
        self.audio_file_path_say_yes_no = os.path.join("audios/repeatYesOrNo.mp3")

        
        # Audios com os comandos para a captura de imagens
        self.audio_file_path_aproximar_da_camera = os.path.join("audios/aproximarDaCamera.mp3")
        self.audio_file_path_vira_rosto_direita = os.path.join("audios/virarRostoDireita.mp3")
        self.audio_file_path_vira_rosto_esquerda = os.path.join("audios/virarRostoEsquerda.mp3")
        
        # Audio do treinamento finalizado
        self.audio_file_path_recognition_finished = os.path.join("audios/recognitionFinished.mp3")

        # Definindo o dicionário de arquivos de áudio
        self.audio_files = {
            "aproximar_da_camera": self.audio_file_path_aproximar_da_camera,
            "vira_rosto_direita": self.audio_file_path_vira_rosto_direita,
            "vira_rosto_esquerda": self.audio_file_path_vira_rosto_esquerda,
        }
        
        self.get_logger().info("Voice node started. Waiting for keyword...")
        self.timer = self.create_timer(2.0, self.listen_for_keyword)  # Timer para repetição do processo

    
    def ask_for_name(self):
        while True:
            self.play_audio(self.audio_file_path_whats_your_name)
            time.sleep(3)
            
            with sr.Microphone() as source:
                self.get_logger().info("Waiting for your name...")
                self.play_audio(self.audio_file_path_waiting)
                audio = self.recognizer.listen(source)

            # Convertendo o áudio capturado e reduzindo o ruído
            audio_data = np.frombuffer(audio.get_raw_data(), np.int16)
            reduced_noise = nr.reduce_noise(y=audio_data, sr=source.SAMPLE_RATE)
            audio_reduced = sr.AudioData(reduced_noise.tobytes(), source.SAMPLE_RATE, 2)

            try:
                name = self.recognizer.recognize_google(audio_reduced, language = 'en')
                self.get_logger().info(f"Received name: {name}")
                
                # Pergunta se o nome está correto
                if self.confirm_name(name):
                    self.publish_name(name)
                    self.proceed_to_next_step()  # Chama a função para continuar os passos
                    break  # Sai do loop se o nome foi confirmado
                else:
                    self.get_logger().info("Name not confirmed. Asking again...")

            except sr.UnknownValueError:
                self.get_logger().info("I didn't catch that. Please, repeat your name.")
                self.play_audio(self.audio_file_path_dont_understand)
                
    def capture_finished_callback(self, msg):
        if msg.data == "finished":
            self.get_logger().info("Capture process has finished.")
            self.play_audio(self.audio_file_path_training_finished)
            self.publish_training_finished()

    def capture_status_callback(self, msg):
        if msg.data == "ok":
            self.capture_status_ok = True
            
            if self.current_step_index < len(self.capture_steps):
                if self.current_step_index == 0:
                    # O primeiro passo já foi reproduzido durante a confirmação do nome
                    self.current_step_index += 1
                    return  # Não faz nada mais aqui, já tocou o áudio do primeiro passo

                current_step = self.capture_steps[self.current_step_index]
                self.play_audio(self.audio_files[current_step])
                self.get_logger().info(f"Playing audio for: {current_step}")

                # Avança para o próximo passo
                self.current_step_index += 1
                
                # Publica 'continue' após cada passo concluído
                self.publish_next_capture_step("continue")
                
                # Se todos os passos foram concluídos
                if self.current_step_index >= len(self.capture_steps):
                    self.get_logger().info("All capture steps completed.")
                    
                    # Adiciona uma pausa de 5 segundos antes de continuar
                    time.sleep(8)

                    # Executa a reprodução do áudio e a publicação em paralelo
                    threading.Thread(target=self.play_audio_and_publish_finished).start()
            else:
                self.get_logger().info("No more steps to perform.")
                self.get_logger().info("Waiting for new instructions...")
                
    def confirm_name(self, name):
        self.speak(f"Did you say your name is {name}? Please, say yes or no")
        
        with sr.Microphone() as source:
            self.get_logger().info("Waiting for confirmation...")
            self.play_audio(self.audio_file_path_waiting)
            audio = self.recognizer.listen(source)
            
        # Convertendo o áudio capturado em um formato que podemos manipular com NumPy
        audio_data = np.frombuffer(audio.get_raw_data(), np.int16)

        # Aplicando a redução de ruído
        reduced_noise = nr.reduce_noise(y=audio_data, sr=source.SAMPLE_RATE)

        # Reconstruindo o áudio com o ruído reduzido
        audio_reduced = sr.AudioData(reduced_noise.tobytes(), source.SAMPLE_RATE, 2)
        
        try:
            response = self.recognizer.recognize_google(audio_reduced, language = 'en').lower()
            
            if "yes" in response:
                self.get_logger().info("User confirmed the name")
                return True
            elif "no" in response:
                self.get_logger().info("User did not confirm the name. Asking again.")
                return False
            else:
                self.get_logger().info("I don't understand. Please say yes or no.")
                self.play_audio(self.audio_file_path_dont_understand)  # Reproduz o áudio de não entendimento
                return self.confirm_name(name)
                
        except sr.UnknownValueError:
            self.get_logger().info("I don't understand you. Please, repeat.")
            self.play_audio(self.audio_file_path_dont_understand)  # Reproduz o áudio de não entendimento
            return self.confirm_name(name)
        
    def listen_for_keyword(self):
        if not self.keyword_detected:
            self.get_logger().info("Say the keyword...")
            
            with sr.Microphone() as source:
                self.get_logger().info("Say the keyword...")
                self.play_audio(self.audio_file_path_waiting)  # Reproduz o áudio de espera
                audio = self.recognizer.listen(source)

            # Convertendo o áudio capturado em um formato que podemos manipular com NumPy
            audio_data = np.frombuffer(audio.get_raw_data(), np.int16)

            # Aplicando a redução de ruído
            reduced_noise = nr.reduce_noise(y=audio_data, sr=source.SAMPLE_RATE)

            # Reconstruindo o áudio com o ruído reduzido
            audio_reduced = sr.AudioData(reduced_noise.tobytes(), source.SAMPLE_RATE, 2)

            try:
                keyword = self.recognizer.recognize_google(audio_reduced, language = 'en')
                
                if "ok google" in keyword.lower() or "okay view" in keyword.lower() or "ok view" in keyword.lower() or "ok bil" in keyword.lower() or 'ok viu' in keyword.lower():
                    keyword = "ok bill"
                    
                self.get_logger().info(f"Captured keyword: {keyword}")

                if "ok bill" in keyword.lower():
                    self.keyword_detected = True  # Marca a palavra-chave como detectada
                    self.get_logger().info(f"Keyword '{keyword}' detected.")
                    self.play_audio(self.audio_file_path_detected)
                    self.ask_for_name()  # Continua para perguntar o nome
                else:
                    self.get_logger().info("Keyword not detected. Trying again.")
                    self.play_audio(self.audio_file_path_dont_understand)  # Reproduz áudio de não entendimento

            except sr.UnknownValueError:
                self.get_logger().info("I did not understand the keyword.")
                self.play_audio(self.audio_file_path_error)  # Reproduz áudio de erro
    
    def person_name_callback(self, msg):
        self.person_name = msg.data  # Armazena o nome publicado
        self.get_logger().info(f"Person name received: {self.person_name}")

    def play_audio(self, audio_file_path):
        if os.path.exists(audio_file_path):
            # Usa o sistema para garantir que a reprodução do áudio bloqueie até o fim
            os.system(f"mpg123 {audio_file_path}")
        else:
            self.get_logger().warning(f"Audio file not found: {audio_file_path}")

    def play_audio_sequence(self):
        for step in self.capture_steps:
            # Espera receber "OK" do tópico /capture_status antes de prosseguir para o próximo passo
            while not self.capture_status_ok:
                rclpy.spin_once(self)  # Processa as mensagens recebidas para verificar o status de captura
            self.capture_status_ok = False  # Reseta o status para aguardar o próximo "OK"

            # Reproduz o áudio correspondente ao passo atual
            self.play_audio(self.audio_files[step])
            self.get_logger().info(f"Playing audio for: {step}")
    
    def play_audio_and_publish_finished(self):
        """Reproduz o áudio de treinamento finalizado e publica o status de treinamento."""
        self.play_audio(self.audio_file_path_training_finished)
        self.publish_training_finished()

    def proceed_to_next_step(self):
        self.get_logger().info("Proceeding to the next steps.")
        
        if self.current_step_index == 0:
            self.play_audio(self.audio_files["aproximar_da_camera"])
            self.publish_next_capture_step("continue")  # Publica o primeiro passo imediatamente
            self.current_step_index += 1  # Avança o índice do passo atual
        # O avanço agora será controlado pelo callback capture_status_callback
        
    def publish_name(self, name):
        msg = String()
        msg.data = name
        self.publisher_name.publish(msg)
        self.get_logger().info(f"Published name: {name}")
            
    def publish_next_capture_step(self, step_name):
        msg = String()
        msg.data = step_name
        self.publisher_next_step.publish(msg)
        self.get_logger().info(f"Published next capture step: {step_name}")
        
    def publish_training_finished(self):
        msg = String()
        msg.data = "round180"
        self.publisher_training_status.publish(msg)
        self.get_logger().info("Published training finished status.")
         
    def recognized_callback(self, msg):
        # Verifica se a mensagem recebida é "recognition completed"
        if msg.data != 'Unknown':
            # Lógica para lidar com o reconhecimento finalizado
            self.get_logger().info("Recognition process has completed.")
            self.play_audio(self.audio_file_path_recognition_finished)

            # Espera 7 segundos e publica o status de treinamento finalizado
            time.sleep(8)
            self.publish_training_finished()
        else:
            self.get_logger().warning(f"Unexpected message received: {msg.data}")

        
    def speak(self, text):  
        # Certifique-se de que o diretório "audios" existe
        if not os.path.exists("audios"):
            os.makedirs("audios")

        tts = gTTS(text=text, lang="en")  # Mantenha o idioma como 'en' ou mude para 'pt' para português
        audio_file = "audios/tts_output.mp3"
        tts.save(audio_file)

        playsound(audio_file)

        os.remove(audio_file)
        

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        voice_node.get_logger().info("Voice node terminated.")
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
