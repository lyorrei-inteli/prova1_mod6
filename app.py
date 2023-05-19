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
        self.get_logger().info("Node iniciado!")

    def send_movement_command(self, x, y):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        self.smd_vel_pub.publish(msg)
        self.get_logger().info("Publish feito! x="+str(x)+"; y="+str(y))


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimMovement()

    movement_queue = [[0.0, 0.5], [0.5, 0.0], [0.0, 0.5], [0.5,0.0], [0.0, 1.0], [1.0, 0.0]]
    movement_stack = []

    try:
        print("Caminho de ida começado!")
        while True:
          movement = movement_queue.pop(0)
          node.send_movement_command(movement[0], movement[1])
          movement_stack.append([movement[0]*-1, movement[1]*-1])
          time.sleep(1)
        
    except:
        print("Caminho de ida finalizado!")

    try:
        print("Caminho de volta começado!")
        while True:
            back_movement = movement_stack.pop()
            node.send_movement_command(back_movement[0], back_movement[1])
            time.sleep(1)
    except:
        print("Caminho de volta finalizado!")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()