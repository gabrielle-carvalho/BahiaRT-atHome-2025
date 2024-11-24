#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        # Publisher para o tópico /cmd_vel
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher para informar o status da navegação
        self.pub_status = self.create_publisher(String, '/status_nav', 10)

        # Subscriber para ouvir o término do cadastramento
        self.sub_cadastro = self.create_subscription(
            String,
            '/cadastro_status',
            self.cadastro_callback,
            10
        )
     
        # Parâmetros de movimento e estado
        self.stage = 0  # Fase do movimento atual
        self.linear_speed = 0.5  # Velocidade linear padrão
        self.side_speed = 0.5  # Velocidade lateral padrão
        self.angular_speed = math.radians(10)  # Velocidade angular em rad/s
        self.permission_granted = False  # Permissão para iniciar o movimento
        self.start_time = None

        # Define o timer para o controle de movimento em 10 Hz
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def cadastro_callback(self, msg):
        # """Callback para processar mensagens do tópico de status do cadastro."""
        if msg.data == "cadastro_concluido":
            self.permission_granted = True
            self.get_logger().info("Cadastro concluído. Navegação pode começar.")
            self.publish_status("Cadastro concluído. Iniciando navegação...")
        else:
            self.get_logger().info(f"Status do cadastro recebido: {msg.data}")

    def control_loop(self):
        # Verifica se a permissão para começar foi concedida
        if not self.permission_granted:
            self.get_logger().info("Aguardando conclusão do cadastro para começar a navegação...")
            return

        # Marca o início da fase de movimento se não estiver inicializado
        if self.start_time is None:
            self.start_time = time.time()

        # Lógica para cada fase de movimento
        if self.stage == 0:  # Rotacionar 180 graus
            self.get_logger().info("Virando em direção à porta...")
            if self.rotate(180):
                self.stage += 1
                self.start_time = None

        elif self.stage == 1:  # Mover para fora da sala
            self.get_logger().info("Saindo da sala onde foi realizado o cadastro...")
            if self.forward(4):
                self.stage += 1
                self.start_time = None

        elif self.stage == 2:  # Mover até o meio da sala
            self.get_logger().info("Indo até o meio da sala...")
            if self.side_mov(4):
                self.stage += 1
                self.start_time = None

        elif self.stage == 3:  # Mover até a frente da mesa
            self.get_logger().info("Indo até a frente da mesa...")
            if self.forward(6):
                self.stage += 1
                self.start_time = None

        elif self.stage == 4:  # Finalizar navegação
            self.get_logger().info("Navegação concluída. Parando o robô.")
            self.stop_robot()
            self.publish_status("Navegação finalizada com sucesso.")
            self.destroy_node()  # Encerra o nó após a navegação

    def forward(self, duration):
        # """Movimento para frente por um determinado tempo."""
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        self.publisher_vel.publish(vel_msg)

        # Checa se a duração foi atingida
        if time.time() - self.start_time >= duration:
            self.stop_robot()
            return True
        return False
    
    def side_mov(self, duration):
        # """Movimento lateral por um determinado tempo."""
        vel_msg = Twist()
        vel_msg.linear.y = self.side_speed  # Corrigido para movimento lateral
        self.publisher_vel.publish(vel_msg)

        # Checa se a duração foi atingida
        if time.time() - self.start_time >= duration:
            self.stop_robot()
            return True
        return False
    
    def rotate(self, angle_degrees):
        # """Rotação por um determinado ângulo."""
        vel_msg = Twist()
        vel_msg.angular.z = self.angular_speed

        # Calcula o tempo necessário para a rotação
        rotation_duration = math.radians(angle_degrees) / self.angular_speed
        self.publisher_vel.publish(vel_msg)

        if time.time() - self.start_time >= rotation_duration:
            self.stop_robot()
            return True
        return False
    
    def stop_robot(self):
        # """Para o movimento do robô."""
        vel_msg = Twist()
        self.publisher_vel.publish(vel_msg)
    
    def publish_status(self, status_msg):
        # """Publica o status da navegação."""
        msg = String()
        msg.data = status_msg
        self.pub_status.publish(msg)
        self.get_logger().info(f"Status publicado: {status_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nó interrompido. Encerrando...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
