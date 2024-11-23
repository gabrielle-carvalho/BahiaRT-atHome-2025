import time
import pandas as pd
from sklearn.model_selection import train_test_split
import torch
from transformers import GPT2Tokenizer, GPT2LMHeadModel, AdamW, get_linear_schedule_with_warmup
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt

# Classe TextDataset para gerenciar o dataset de textos
class TextDataset(Dataset):
    def __init__(self, df, tokenizer, max_length=256):
        self.texts = df['text'].values
        self.tokenizer = tokenizer
        self.max_length = max_length

    def __len__(self):
        return len(self.texts)

    def __getitem__(self, idx):
        inputs = self.tokenizer(self.texts[idx], return_tensors='pt', padding='max_length', max_length=self.max_length, truncation=True)
        inputs = {key: val.squeeze(0) for key, val in inputs.items()}  # Remove dimensões extras
        inputs['labels'] = inputs['input_ids'].clone()
        return inputs

# Expandindo o DataFrame com contextos e perguntas
questions = [
    "What is the hero’s name in The Legend of Zelda?",
    "What are the names of the ghosts who chase Pac Man and Ms. Pac Man?",
    "What’s the name of the Mythbusters’ crash test dummy?",
    "What is an Oxford comma?",
    "Who was the captain of the Enterprise in the pilot episode of Star Trek?",
    "What is the symbol for the modulus operator in C?",
    "What function is automatically called at the beginning of a C++ program?",
    "Which computer programming languages was introduced by IBM in 1957?",
    "Who is considered as the first programmer?",
    "Has a robot ever killed a person?",
    "Who was HitchBOT?",
    "Are self-driving cars safe?",
    "Who invented the compiler?",
    "Who created the Python Programming Language?",
    "Is Mark Zuckerberg a robot?",
    "Why did you run away?",
    "What kind of salad do robots like?",
    "What did you ate for lunch?",
    "Why did robots get angry so often?",
    "Why shouldn’t R2D2 be allowed in movies?",
    "What’s your favorite style of music?",
    "What does the acronym SMTP represent?",
    "What does the acronym MPEG represent?",
    "What does the acronym GIMP represent?",
    "What does the acronym GNU represent?",
    "What is the most populous city in Brazil?",
    "Which continent is Brazil located in?",
    "On what day, month and year was Brazil’s independence de- clared?",
    "How many states does Brazil have (with Federal District)?",
    "In what year did the first Brazilian astronaut go to space?",
    "What is the only capital of Brazil crossed by the Equator?",
    "How many time zones are there in Brazil?",
    "In which city is the world’s first urban elevator and what is the name of that elevator?",
    "What is the only biome present in Brazil that is exclusive in the world?",
    "Pampulha Lake is a tourist spot in which Brazilian city?",
    "What is the smallest Brazilian state in territorial extension?",
    "Which capitals in Brazil have the same name as your state?",
    "Where is the Itamaraty Palace located?",
    "What was the first name given to Brazil by the Portuguese?",
    "What is the Newest State in Brazil?",
    "What is the oldest state in Brazil?",
    "What is the capital of Ceará?",
    "What is the capital of Rio Grande do Sul?",
    "What is the capital of Rio Grande do Norte?",
    "What is the capital of Brazil?",
    "What is the capital of Pernambuco?",
    "What is the capital of Pará?",
    "What is the capital of Bahia?",
    "Acarajé is a typical food from which state?",
    "What are the colors of Bahia's flag?",
    "What are some typical foods from Bahia?",
    "What is the most popular rhythm in Bahia, played mainly during the Bahian Carnival?"
]
contexts = [
    "Despite most people’s believes, he’s called Link",
    "Inky, Blinky, Pinky, and Clyde",
    "The Mythbusters’ crash test dummy is called Buster",
    "The hotly contested punctuation before a conjunction in a list",
    "The captain of the Enterprise in the pilot episode was Captain Pike",
    "The percentage symbol is used as modulus operator in C",
    "The main function",
    "Fortran was introduced by IBM in 1957",
    "The first programmer was Ada Lovelace",
    "The first known case of robot homicide occurred in 1981, when a robotic arm crushed a Japanese Kawasaki factory worker",
    "A hitchhiking robot that relied on the kindness of strangers to travel the world and was slain by humans",
    "Yes. Car accidents are product of human misconduct",
    "Grace Hoper. She wrote it in her spare time",
    "Python was invented by Guido van Rossum",
    "Sure. I’ve never seen him drink water",
    "I heard an electric can opener",
    "Salads made with ice-borg lettuce.",
    "I had a byte",
    "People kept pushing our buttons.",
    "He says so many foul words they have to bleep everything he says!",
    "I like electronic and heavy Metal",
    "SMTP stands for Simple Mail Transport Protocol",
    "MPEG stands for Moving Picture Experts Group",
    "GNU Image Manipulation Program",
    "GNU is a recursive acronym meaning GNU is Not Unix",
    "São Paulo is the most populous city in Brazil with 12.03 million residents.",
    "The Brazilian territory is located on the South American continent.",
    "On September 7, 1822, Brazil’s indexpendence was declared.",
    "Currently, Brazil is divided into 26 states and the Federal District, altogether there are 27 federative units.",
    "In March 2006, Pontes became the first Brazilian to go to space.",
    "Macapá is the only Brazilian capital crossed by the Equator line.",
    "Brazil is a country with continental dimensions, in all it has four time zones.",
    "The Lacerda Elevator is a public urban elevator located in Salvador, Brazil.",
    "The Caatinga, characterized by its dry, desert habitat is the only one of Brazil’s biomes found exclusively within the country.",
    "Belo Horizonte",
    "Sergipe",
    "São Paulo and Rio de Janeiro",
    "Brasília",
    "Ilha de vera Cruz",
    "Tocantins",
    "Pernambuco",
    "Fortaleza",
    "Porto alegre",
    "Natal",
    "Brasília",
    "Recife",
    "Belém",
    "Salvador",
    "Bahia",
    "White, red and blue",
    "Caruru, vatapá, abará and acarajé.",
    "Axé or pagode."
]

labels = ["General (simple) questions"] * 21 + \
         ["General (complex) questions"] * 4 + \
         ["Questions about Brazil (simple)"] * 21 + \
         ["Questions about Bahia (simple)"] * 5

# Verifique os comprimentos das listas
print("Comprimento de labels:", len(labels))
print("Comprimento de questions:", len(questions))
print("Comprimento de contexts:", len(contexts))

# Ajuste as listas para o comprimento máximo
max_length = min(len(labels), len(questions), len(contexts))
labels = labels[:max_length]
questions = questions[:max_length]
contexts = contexts[:max_length]

# Criação do DataFrame combinando perguntas e contextos
questions_contexts = pd.DataFrame({
    "label": labels,
    "text": [f"{question} {context}" for question, context in zip(questions, contexts)]
})

# Se não houver um DataFrame existente, use diretamente `questions_contexts`
df = questions_contexts.copy()

# Definindo o número de vezes que o DataFrame deve ser replicado
n_copies = 10  # Defina o número de vezes que você deseja replicar o DataFrame

# Concatenando com o DataFrame existente
df = pd.concat([df] * n_copies, ignore_index=True)

# Dividir em conjuntos de treinamento e teste
train_df, test_df = train_test_split(df, test_size=0.2)

# Tokenizador e modelo
tokenizer = GPT2Tokenizer.from_pretrained('gpt2')
model = GPT2LMHeadModel.from_pretrained('gpt2')

tokenizer.pad_token = tokenizer.eos_token

# Dataset e DataLoader
train_dataset = TextDataset(train_df, tokenizer, max_length=256)  # Aumentado o comprimento máximo
train_dataloader = DataLoader(train_dataset, batch_size=8, shuffle=True)

# Otimizador e scheduler
optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=0.05)
scheduler = get_linear_schedule_with_warmup(optimizer, num_warmup_steps=0, num_training_steps=len(train_dataloader) * 15)

# Cronometrar tempo do treinamento
start_time = time.time()

# Treinamento
losses = []
lowest_loss = float('inf')

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model.to(device)

model.train()
for epoch in range(15):
    total_loss = 0
    for batch in train_dataloader:
        optimizer.zero_grad()
        inputs = {key: val.to(device) for key, val in batch.items()}
        outputs = model(**inputs)
        loss = outputs.loss
        loss.backward()
        optimizer.step()
        scheduler.step()
        total_loss += loss.item()

    avg_loss = total_loss / len(train_dataloader)
    losses.append(avg_loss)
    
    # Salvando o melhor modelo
    if avg_loss < lowest_loss:
        lowest_loss = avg_loss
        model.save_pretrained("fine_tuned_gpt2")
        tokenizer.save_pretrained("fine_tuned_gpt2")

    print(f"Epoch {epoch + 1}, Loss: {avg_loss}")

# Tempo total de treinamento
total_time = time.time() - start_time
hours, rem = divmod(total_time, 3600)
minutes, seconds = divmod(rem, 60)
print(f"Tempo total de treinamento: {int(hours)}h {int(minutes)}min {int(seconds)}s")

# Plotando a função de perda
plt.plot(losses)
plt.xlabel("Época")
plt.ylabel("Loss")
plt.title("Loss durante o treinamento")
plt.show()