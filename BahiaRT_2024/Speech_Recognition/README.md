# BahiaRT_atHome

## Speech Recognition - Context

Este projeto implementa um sistema de *Question Answering* (QA) utilizando ROS2. Ele é composto por dois módulos principais:  
1. Responder perguntas com base em um **contexto predefinido** usando um modelo **DistilBERT**.  
2. Treinamento do modelo DistilBERT a partir do arquivo `context.txt`.  

---

## Pré-requisitos

Certifique-se de que seu sistema tenha as seguintes dependências instaladas:  

## Ferramentas Gerais
- Python 3.8+  
- ROS2 (Humble)  
- `pip` para gerenciar pacotes Python  

## Bibliotecas Python
Instale as bibliotecas necessárias executando:  
```bash
pip install -r requirements.txt

```
## Configuração

1. Clone este repositório:
   ```bash
   git clone https://github.com/gabrielle-carvalho/BahiaRT_atHome.git
   cd BahiaRT_atHome

2. Compile o pacote ROS2
   ```bash
   colcon build
3. Configurar o Ambiente do Workspace
   ```bash
   source install/setup.bash
4. Configure o contexto:

   Se necessário insira o texto do contexto no arquivo `speech_pkg/context.txt`. Este será usado para responder perguntas.

---

## Como Executar

1. Execute o arquivo de lançamento para iniciar o nó principal:
   ```bash
   ros2 launch speech_pkg speech_launch.py
   ```
2. Interagir com o Sistema após o nó ser iniciado:

- Fale uma pergunta em inglês para o microfone.
- O sistema processará a fala, identificará a pergunta e fornecerá uma resposta com base no contexto carregado

3. Treinamento do Modelo
Caso deseje treinar o modelo DistilBERT com um novo contexto, siga os passos abaixo:

- Edite o arquivo `speech_pkg/context.txt` com o novo contexto.
- Execute o script de treinamento:
  ```bash
  python3 bert_treinamento.py
