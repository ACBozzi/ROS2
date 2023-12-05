import rclpy
from geometry_msgs.msg import Twist

def move_turtlebot3():
    rclpy.init()  # Inicializa o sistema ROS2
    node = rclpy.create_node('controlador_turtlebot3')  # Cria um nó ROS2 com o nome 'controlador_turtlebot3'

    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)  # Cria um publicador para o tópico /cmd_vel

    # Cria uma mensagem Twist para enviar comandos de velocidade
    twist = Twist()

    # Define velocidades linear e angular desejadas
    twist.linear.x = 0.2  # Ajuste a velocidade linear conforme necessário
    twist.angular.z = 0.5  # Ajuste a velocidade angular conforme necessário

    # Loop principal para enviar comandos de velocidade continuamente
    while rclpy.ok():  # Enquanto o sistema ROS2 estiver operacional
        cmd_vel_publisher.publish(twist)  # Publica a mensagem Twist no tópico /cmd_vel
        node.get_logger().info('Movendo Turtlebot3')  # Mensagem de log indicando que o Turtlebot3 está em movimento
        node.get_logger().info('Velocidade Linear: {}, Velocidade Angular: {}'.format(
            twist.linear.x, twist.angular.z))  # Imprime as velocidades linear e angular

        # Desenvolver lógica dos obstáculos 

        # Realiza um ciclo do sistema ROS2
        rclpy.spin_once(node, timeout_sec=0.1)  # Aguarda um curto intervalo de tempo para evitar bloqueios

    node.destroy_node()  # Finaliza o nó ROS2
    rclpy.shutdown()  # Encerra o sistema ROS2

if __name__ == '__main__':
    move_turtlebot3()  

