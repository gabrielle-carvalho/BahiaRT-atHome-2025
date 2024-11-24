#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

# class TurnAround(Node):
#     def __init__(self):
#         super().__init__("turn_around_node")
        
#         self.turned_publisher = self.create_publisher(String, 'turned_around', 10)
#         self.get_logger().info("Node has started")
        
#         self.timer = self.create_timer(2.0, self.publish_message)

#     def publish_message(self):
#         msg = String()
#         msg.data = "Turned around"
#         self.turned_publisher.publish(msg)
#         self.get_logger().info(f"Published: {msg.data}")

# def main(args=None):
#     rclpy.init(args=args)
    
#     node = TurnAround()
#     rclpy.spin(node)

#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('turn_around_node')

        # Cria o publisher no tópico /cmd_vel
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Cria o publisher no tópico /turned_around
        self.publisher_fin_round = self.create_publisher(String, 'turned_around', 10)

        #subscriber para escutar o topico rotatetopicpy
        self.subscription = self.create_subscription(String, 'training_finished', self.listener_callback, 10)

        
        # Define a taxa de publicação (por exemplo, 10 Hz)
        self.timer_period = 0.1  # Segundos (10 Hz)
        self.timer = None  # O timer só será criado quando receber o "round180"

        # Cria uma mensagem Twist para controlar a velocidade
        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 0.0  # Velocidade linear no eixo X (frente)
        self.velocity_msg.linear.y = 0.0
        self.velocity_msg.linear.z = 0.0
        self.velocity_msg.angular.x = 0.0
        self.velocity_msg.angular.y = 0.0
        self.velocity_msg.angular.z = 0.0
        
        self.elapsed_time = 0.0  # Tempo decorrido em segundos
        self.duration = 2.5  # Duração em segundos para ir para frente
        self.is_rotating = False
    
    def listener_callback(self, msg):
        if msg.data == 'round180' and not self.is_rotating:
            self.is_rotating = True
            self.get_logger().info('Recebido comando "round180"')
            self.get_logger().info('Aguardando 60 segundos...')

            for i in range(10):
                print(10-i)
                time.sleep(1)

            # Configura a velocidade angular para girar
            self.velocity_msg.linear.y = -0.7

            # Cria um timer para começar a publicar a velocidade
            self.timer = self.create_timer(self.timer_period, self.publish_velocity)


    def publish_velocity(self):
        if self.elapsed_time <= self.duration:
            # Publica a mensagem Twist no tópico /cmd_vel
            self.publisher_vel.publish(self.velocity_msg)
            self.get_logger().info('Publicando: linear.x: %.2f' % (self.velocity_msg.linear.x))
            self.get_logger().info('Publicando: linear.y: %.2f' % (self.velocity_msg.linear.y))
            self.get_logger().info('Publicando: linear.z: %.2f' % (self.velocity_msg.linear.z))

            self.get_logger().info('Publicando: angular.x: %.2f' % (self.velocity_msg.angular.x))
            self.get_logger().info('Publicando: angular.y: %.2f' % (self.velocity_msg.angular.y))
            self.get_logger().info('Publicando: angular.z: %.2f' % (self.velocity_msg.angular.z))
            self.get_logger().info(' ')

            self.get_logger().info('Tempo: %.2f' % (self.elapsed_time))
            self.elapsed_time += self.timer_period
        else:
            # Para o robô
            self.velocity_msg.linear.x = 0.0
            self.velocity_msg.linear.y = 0.0
            self.velocity_msg.linear.z = 0.0

            self.velocity_msg.angular.x = 0.0
            self.velocity_msg.angular.y = 0.0
            self.velocity_msg.angular.z = 0.0

            self.publisher_vel.publish(self.velocity_msg)  # Publica a parada
            self.get_logger().info('Parando o robô.')
            fin_msg = String()
            fin_msg.data = 'finalizado'
            self.publisher_fin_round.publish(fin_msg)
            self.get_logger().info('Publicando a mensagem de finalização de giro')
            
            self.timer.cancel()  # Para o timer
            self.destroy_node()  # Destroi o nó
            rclpy.shutdown()  # Encerra o ROS

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()

    try:
        # Mantém o nó rodando até que seja interrompido
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted. Shutting down...")
    finally:
        rclpy.shutdown()

    # O nó será destruído e o ROS será encerrado dentro do método publish_velocity

if __name__ == '__main__':
    main()