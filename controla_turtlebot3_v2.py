import rclpy #Biblioteca principal para trabalhar com ROS2 em Python.
from gazebo_msgs.msg import ModelStates #Mensagem que fornece informações sobre o estado de modelos no ambiente Gazebo.
#Twist: Mensagem que representa comandos de velocidade linear e angular.
#Pose: Mensagem que representa a posição e orientação de um objeto no espaço tridimensional.
from geometry_msgs.msg import Twist, Pose  
from sensor_msgs.msg import LaserScan #Mensagem que contém dados do sensor de laser.
from rclpy.qos import QoSProfile # Perfil de qualidade de serviço para definir a qualidade da comunicação ROS.
import random

position = Pose() # Variável global para armazenar a pose do Turtlebot3.
laser = LaserScan() #Variável global para armazenar os dados do sensor de laser.


#Atualiza a variável global position com a última pose recebida.
def position_callback(data):
    global position
    position = data.pose[-1]

#tualiza a variável global laser com os dados mais recentes do sensor de laser.
def laser_callback(data):
    global laser
    laser = data  

def move_turtlebot3(node):
    #rclpy.init()  # Inicializa o sistema ROS2
    node = rclpy.create_node('controlador_turtlebot3')  # Cria um nó ROS2 com o nome 'controlador_turtlebot3'

    # Subscribers que recebem os dados do Gazebo
    node.create_subscription(ModelStates, '/gazebo/model_states', position_callback, QoSProfile(depth=10))
    node.create_subscription(LaserScan, '/scan', laser_callback, QoSProfile(depth=10))

    # Publisher para enviar os comandos de velocidade
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)  # Cria um publicador para o tópico /cmd_vel

    # Cria uma mensagem Twist para enviar comandos de velocidade
    twist = Twist()

    # Loop principal para enviar comandos de velocidade continuamente
    while rclpy.ok():  # Enquanto o sistema ROS2 estiver operacional
        if (len(laser.ranges) > 0):
            if (min(laser.ranges[90:270]) > 0.35):
                # Lógica para evitar obstáculos
                twist.angular.z = 0.0
                twist.linear.x = random.uniform(-0.25, -0.35) # ajusta a velocidade linear negativa de forma aleatória
            else:
                # Se não houver obstáculos significativos à frente, use as velocidades definidas originalmente
                twist.linear.x = 0.2
                twist.angular.z = 0.5
                
        cmd_vel_publisher.publish(twist)  # Publica a mensagem Twist no tópico /cmd_vel
        node.get_logger().info('Movendo Turtlebot3')  # Mensagem de log indicando que o Turtlebot3 está em movimento
        node.get_logger().info('Velocidade Linear: {}, Velocidade Angular: {}'.format(
            twist.linear.x, twist.angular.z))  # Imprime as velocidades linear e angular

        rclpy.spin_once(node, timeout_sec=0.01)  # Realiza um ciclo do sistema ROS2

    node.destroy_node()  # Finaliza o nó ROS2
    rclpy.shutdown()  # Encerra o sistema ROS2

if _name_ == '_main_':
    rclpy.init()  # Inicializa o sistema ROS2
    node = rclpy.create_node('controlador_turtlebot3')  # Cria um nó ROS2 com o nome 'controlador_turtlebot3'
    
    try:
        move_turtlebot3(node)
    finally:
        node.destroy_node()  # Finaliza o nó ROS2
        rclpy.shutdown()  # Encerra o sistema ROS2