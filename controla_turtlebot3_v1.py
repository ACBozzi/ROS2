import rclpy
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
import random

position = Pose()
laser = LaserScan()

def position_callback(data):
    global position
    position = data.pose[-1]

def laser_callback(data):
    global laser
    laser = data  

def move_turtlebot3():
    rclpy.init()  # Inicializa o sistema ROS2
    node = rclpy.create_node('controlador_turtlebot3')  # Cria um nó ROS2 com o nome 'controlador_turtlebot3'

    # Subscribers
    node.create_subscription(ModelStates, '/gazebo/model_states', position_callback, QoSProfile(depth=10))
    node.create_subscription(LaserScan, '/scan', laser_callback, QoSProfile(depth=10))

    # Publisher
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)  # Cria um publicador para o tópico /cmd_vel

    # Cria uma mensagem Twist para enviar comandos de velocidade
    twist = Twist()

    # Loop principal para enviar comandos de velocidade continuamente
    while rclpy.ok():  # Enquanto o sistema ROS2 estiver operacional
        if (len(laser.ranges) > 0):
            if (min(laser.ranges[90:270]) > 0.35):
                # Lógica para evitar obstáculos
                twist.angular.z = 0.0
                twist.linear.x = random.uniform(-0.25, -0.35)
            else:
                # Se não houver obstáculos significativos à frente, use as velocidades definidas originalmente
                twist.linear.x = 0.2
                twist.angular.z = 0.5
                
        cmd_vel_publisher.publish(twist)  # Publica a mensagem Twist no tópico /cmd_vel
        node.get_logger().info('Movendo Turtlebot3')  # Mensagem de log indicando que o Turtlebot3 está em movimento
        node.get_logger().info('Velocidade Linear: {}, Velocidade Angular: {}'.format(
            twist.linear.x, twist.angular.z))  # Imprime as velocidades linear e angular

        rclpy.spin_once(node, timeout_sec=0.1)  # Realiza um ciclo do sistema ROS2

    node.destroy_node()  # Finaliza o nó ROS2
    rclpy.shutdown()  # Encerra o sistema ROS2

if __name__ == '__main__':
    move_turtlebot3()

