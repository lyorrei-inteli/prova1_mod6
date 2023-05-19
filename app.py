#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


# Node que vai fazer o publish para movimentar o Turtlesim
class TurtlesimMovement(Node):
    def __init__(self):
        # Nome do node
        super().__init__("turtlesim_movement")

        # Setup do publisher
        self.smd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Printando que tudo o node foi iniciado
        self.get_logger().info("Node iniciado!")

    # Função que vai mandar os comandos de movimento da tartaruga
    def send_movement_command(self, x, y):
        # Criando a mensagem com informações de movimento do tipo Twist
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y

        # Publicando no tópico mandando a mensagem criada
        self.smd_vel_pub.publish(msg)

        # Printando que a publicação foi bem sucedida
        self.get_logger().info("Publish feito! x=" + str(x) + "; y=" + str(y))


# Função main que vai ser executada
def main(args=None):
    # Setup do rlcpy
    rclpy.init(args=args)

    # Instância do Node que movimenta a tartaruga
    node = TurtlesimMovement()

    # Fila com os movimentos dados no enunciado da atividade
    movement_queue = [
        [0.0, 0.5],
        [0.5, 0.0],
        [0.0, 0.5],
        [0.5, 0.0],
        [0.0, 1.0],
        [1.0, 0.0],
    ]

    # Stack que vai ser preenchida com os movimentos inversos
    movement_stack = []

    # Iniciando a movimentação de ida do turtlesim
    try:
        print("Caminho de ida começado!")

        # Enquanto existir um caminho na fila, executar o movimento
        while True:
            # Retirando o primeiro movimento da fila e colocando em uma variável
            movement = movement_queue.pop(0)

            # Mandando o comando de movimento
            node.send_movement_command(movement[0], movement[1])

            # Adicionando as informações do movimento inverso na stack de movimentos inversos
            movement_stack.append([movement[0] * -1, movement[1] * -1])

            # Esperando a movimentação acontecer
            time.sleep(1)

    except:
        print("Caminho de ida finalizado!")

    # Iniciando a movimentação de volta do turtlesim
    try:
        print("Caminho de volta começado!")

        # Enquanto existir um caminho no stack, executar o movimento
        while True:
            # Retirando o primeiro movimento do stack e colocando em uma variável
            back_movement = movement_stack.pop()

            # Mandando o comando de movimento
            node.send_movement_command(back_movement[0], back_movement[1])

            # Esperando a movimentação acontecer
            time.sleep(1)
    except:
        print("Caminho de volta finalizado!")

    # Deixando o node ligado
    rclpy.spin(node)

    # Desligando o ambiente rclpy
    rclpy.shutdown()


# Executando a função main
if __name__ == "__main__":
    main()
