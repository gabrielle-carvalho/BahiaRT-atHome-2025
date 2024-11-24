#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from transformers import GPT2LMHeadModel, GPT2Tokenizer
import torch
import speech_recognition as sr
from gtts import gTTS
import os
import playsound
import noisereduce as nr
import numpy as np

# Constante para o nome do arquivo de histórico de perguntas e respostas
FILE_NAME = "answer_qa.txt"

class QAAssistantNode(Node):
    def __init__(self):
        super().__init__('qa_assistant_node')
        
        # Carregar o modelo e o tokenizador GPT-2
        model_dir = "fine_tuned_gpt2"
        self.model, self.tokenizer = self.load_model_and_tokenizer(model_dir)
        
        # Inicializar o reconhecimento de fala
        self.recognizer = sr.Recognizer()

        # Palavra-chave para ativação do assistente
        self.keywords = ["Ok Bill"]
        
        self.get_logger().info("QA Assistant Node has been initialized.")

    def load_model_and_tokenizer(self, model_dir):
        model = GPT2LMHeadModel.from_pretrained(model_dir)
        tokenizer = GPT2Tokenizer.from_pretrained(model_dir)
        return model, tokenizer

    def generate_text(self, prompt, max_length=150, temperature=0.7, top_p=0.9):
        input_ids = self.tokenizer.encode(prompt, return_tensors='pt')
        with torch.no_grad():
            output = self.model.generate(
                input_ids,
                max_length=max_length,
                num_return_sequences=1,
                no_repeat_ngram_size=3,
                temperature=temperature,
                top_p=top_p,
                pad_token_id=self.tokenizer.eos_token_id,
                do_sample=True
            )

        generated_text = self.tokenizer.decode(output[0], skip_special_tokens=True).strip()
        if generated_text.startswith(prompt):
            generated_text = generated_text[len(prompt):].strip()
        return generated_text

    def save_response(self, question, answer):
        with open(FILE_NAME, 'a') as file:
            file.write(f"Question: {question}\n")
            file.write(f"Answer: {answer}\n")
            file.write("\n")

    def recognize_speech_from_audio(self):
        with sr.Microphone() as source:
            self.get_logger().info("Ajustando o ruído ambiente. Por favor, aguarde...")
            self.recognizer.adjust_for_ambient_noise(source)
            
            self.get_logger().info("Por favor, faça sua pergunta...")
            playsound.playsound("audios/askQuestion.mp3")
            playsound.playsound("audios/microphoneOn.mp3")
            
            audio = self.recognizer.listen(source)
            audio_data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
            reduced_noise = nr.reduce_noise(y=audio_data, sr=source.SAMPLE_RATE)
            audio = sr.AudioData(reduced_noise.tobytes(), source.SAMPLE_RATE, audio.sample_width)
        
        try:
            return self.recognizer.recognize_google(audio, language="en")
        except sr.UnknownValueError:
            self.get_logger().info("Não entendi o que você disse. Por favor, repita.")
            playsound.playsound("audios/dontUnderstand.mp3")
            return None
        except sr.RequestError as e:
            self.get_logger().info(f"Erro ao se conectar com o serviço de reconhecimento de fala: {e}")
            return None

    def play_answer(self, answer_text):
        if answer_text.strip():
            tts = gTTS(text=answer_text, lang="en")
            audio_file = "answer.mp3"
            tts.save(audio_file)
            playsound.playsound(audio_file)
            os.remove(audio_file)
        else:
            self.get_logger().info("Resposta vazia gerada. Não é possível reproduzir o áudio.")

    def wait_for_keyword(self):
        while True:
            with sr.Microphone() as source:
                self.get_logger().info("Ajustando o ruído ambiente. Por favor, aguarde...")
                self.recognizer.adjust_for_ambient_noise(source)
                
                self.get_logger().info("Aguardando palavra-chave...")
                playsound.playsound("audios/microphoneOn.mp3")
                audio = self.recognizer.listen(source)
            
            try:
                text = self.recognizer.recognize_google(audio, language='en')
                if any(keyword.lower() in text.lower() for keyword in self.keywords):
                    self.get_logger().info("Palavra-chave detectada!")
                    playsound.playsound("audios/keywordDetected.mp3")
                    return True
                else:
                    self.get_logger().info("Palavra-chave não detectada. Por favor, tente novamente.")
                    playsound.playsound("audios/erro.mp3")
            except sr.UnknownValueError:
                self.get_logger().info("Não entendi o que você disse. Por favor, repita.")
                playsound.playsound("audios/dontUnderstand.mp3")
            except sr.RequestError as e:
                self.get_logger().info(f"Erro ao solicitar ao serviço de reconhecimento de fala: {e}")

    def main_loop(self):
        while rclpy.ok():
            if self.wait_for_keyword():
                question = self.recognize_speech_from_audio()
                if question:
                    self.get_logger().info(f"Pergunta reconhecida: {question}")
                    
                    prompt = f"Answer the following question: {question}"
                    answer = self.generate_text(prompt)
                    
                    self.get_logger().info(f"Resposta gerada: {answer}")
                    self.save_response(question, answer)
                    
                    if answer.strip():
                        self.play_answer(answer)
                    else:
                        self.get_logger().info("O modelo não gerou uma resposta válida.")

def main(args=None):
    rclpy.init(args=args)
    node = QAAssistantNode()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()