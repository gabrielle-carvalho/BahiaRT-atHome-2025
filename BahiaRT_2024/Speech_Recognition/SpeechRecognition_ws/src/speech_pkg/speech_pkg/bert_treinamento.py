import os
from transformers import DistilBertForQuestionAnswering, DistilBertTokenizer
from ament_index_python.packages import get_package_share_directory

def generate_training_file():
    package_share_directory = get_package_share_directory('speech_pkg')

    # Carregar o modelo e tokenizer
    model = DistilBertForQuestionAnswering.from_pretrained("distilbert-base-uncased-distilled-squad")
    tokenizer = DistilBertTokenizer.from_pretrained("distilbert-base-uncased")

    # Carregar o contexto do arquivo "context.txt"
    context_file_path = os.path.join(package_share_directory, "context.txt")
    with open(context_file_path, 'r', encoding='utf-8') as file:
        context = file.read()

    # Salvar o modelo treinado
    model.save_pretrained("fine_tuned_bert")
    tokenizer.save_pretrained("fine_tuned_bert")

    print(f"Arquivo de contexto lido de: {context_file_path}")
    print("Modelo e tokenizer salvos em 'fine_tuned_bert'.")

if __name__ == "__main__":
    generate_training_file()
