#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from transformers import DistilBertForQuestionAnswering, DistilBertTokenizer
import torch
import speech_recognition as sr
from gtts import gTTS
import os
import playsound
import noisereduce as nr
import numpy as np
from ament_index_python.packages import get_package_share_directory
from fuzzywuzzy import process, fuzz
import time
class SpeechContextNode(Node):
    def __init__(self):
        super().__init__('speech_context_node')

        # Inicializa componentes
        self.get_logger().info("Iniciando o nó de reconhecimento de fala e QA.")
        self.recognizer = sr.Recognizer()
        self.model = DistilBertForQuestionAnswering.from_pretrained("distilbert-base-uncased-distilled-squad")
        self.tokenizer = DistilBertTokenizer.from_pretrained("distilbert-base-uncased")

        # Diretório do pacote
        package_share_directory = get_package_share_directory('speech_pkg')

        # Carregar o contexto
        context_file_path = os.path.join(package_share_directory, "context.txt")
        self._context = self.read_file(context_file_path)
        self.file_name = "answer.txt"

        # Caminhos dos arquivos de áudio
        self.audio_ask_question = os.path.join(package_share_directory, "audios", "askQuestion.mp3")
        self.audio_microphone_on = os.path.join(package_share_directory, "audios", "microphoneOn.mp3")
        self.audio_dont_understand = os.path.join(package_share_directory, "audios", "dontUnderstand.mp3")

    

        # Se necessário, adicione os outros aúdios, apenas coloquei os que deram erro

        if self._context is None:
            self.get_logger().error("Falha ao carregar o contexto do arquivo. Encerrando o nó.")
            return  # Apenas retorna, sem chamar rclpy.shutdown aqui

    def read_file(self, file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                context = file.read()
            return context
        except FileNotFoundError:
            self.get_logger().error(f"O arquivo '{file_path}' não foi encontrado.")
            return None
        except IOError as e:
            self.get_logger().error(f"Ocorreu um erro de I/O: {e}")
            return None

    def save_response(self, question, answer):
        with open(self.file_name, 'a') as file:
            file.write(f"Question: {question}\n")
            file.write(f"Answer: {answer}\n")
            file.write("\n")

    def generate_answer(self, question):
        if not self._context or not question:
            return "Desculpe, não consegui processar sua solicitação."

        # Divida o contexto em partes menores (sentenças, por exemplo)
        sentences = self._context.split('.')
        
        # Encontre a sentença mais relevante usando fuzzy matching
        best_match = process.extractOne(question, sentences, scorer=fuzz.token_sort_ratio)
        
        if best_match:
            best_sentence = best_match[0]  # Sentença mais relevante
            self.get_logger().info(f"Sentença mais relevante: {best_sentence}")
        else:
            return "Desculpe, não encontrei uma resposta adequada."

        # Utilize o modelo DistilBERT para gerar a resposta com base na sentença encontrada
        inputs = self.tokenizer.encode_plus(question, best_sentence, return_tensors='pt', truncation=True, max_length=512)
        input_ids = inputs["input_ids"].squeeze().tolist()

        outputs = self.model(**inputs)
        answer_start = torch.argmax(outputs.start_logits)
        answer_end = torch.argmax(outputs.end_logits) + 1

        answer = self.tokenizer.convert_tokens_to_string(self.tokenizer.convert_ids_to_tokens(input_ids[answer_start:answer_end]))
        return answer.strip()


    def recognize_speech(self):
        with sr.Microphone() as source:
            self.get_logger().info("Ajustando o ruído ambiente. Por favor, aguarde...")
            self.recognizer.adjust_for_ambient_noise(source)

            self.get_logger().info("Por favor, faça sua pergunta...")
            playsound.playsound(self.audio_ask_question)
            playsound.playsound(self.audio_microphone_on)

            # Aumentando o tempo de escuta com phrase_time_limit e ajustando pause_threshold
            self.recognizer.pause_threshold = 5.5  # Ajuste para aumentar a tolerância de pausas
            audio = self.recognizer.listen(source, phrase_time_limit=50)  # Aumenta o tempo máximo de escuta para 10 segundos

            audio_data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
            reduced_noise = nr.reduce_noise(y=audio_data, sr=source.SAMPLE_RATE)
            audio = sr.AudioData(reduced_noise.tobytes(), source.SAMPLE_RATE, audio.sample_width)

        try:
            self.get_logger().info("Reconhecendo...")
            return self.recognizer.recognize_google(audio, language="en")
        except sr.UnknownValueError:
            self.get_logger().warn("Não entendi o que você disse. Por favor, repita.")
            playsound.playsound(self.audio_dont_understand)
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"Erro ao conectar ao serviço de reconhecimento: {e}")
            return None

    def play_answer(self, answer):
        if answer.strip():
            tts = gTTS(text=answer, lang="en")
            audio_file = "answer.mp3"
            tts.save(audio_file)
            playsound.playsound(audio_file)
            os.remove(audio_file)
        else:
            self.get_logger().warn("Resposta vazia gerada. Não é possível reproduzir o áudio.")

    def main_loop(self):
        if self._context is None:
            return  # Sai do loop se o contexto não foi carregado corretamente
        while rclpy.ok():
            question = self.recognize_speech()
            if question is None:
                continue

            self.get_logger().info(f"Pergunta reconhecida: {question}")
            answer = self.generate_answer(question)
            self.get_logger().info(f"Resposta gerada: {answer}")

            self.save_response(question, answer)
            self.play_answer(answer)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechContextNode()

    try:
        if node._context is not None:  # Verifica se o contexto foi carregado corretamente antes de rodar o loop principal
            node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



