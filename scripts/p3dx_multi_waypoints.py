#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations  # Para converter quaternion → Euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import threading
import sys
import tty
import termios
import select

pub = None
pose_bot = (0, 0)
orientation_bot = 0
sensor_ranges = None
sensor_angle_min = -2.09
sensor_angle_max =  2.09

# Lista de waypoints (hard code)
WAYPOINTS = [(-1, -3), (-1, 0), (1.5, 0)]

# Variável global para controle de emergência
emergency_stop = False

def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_pose():
    return pose_bot

def error_distance_robot(goal):
    x1, y1 = get_pose()
    x2, y2 = goal
    euclidean_distance(x1, y1, x2, y2)
    return 0

def find_obstacles(pose=(0, 0), theta=0.0, ranges=None, sensor_angle_min=0, sensor_angle_max=6.28, max_distance=10):
    """
    Encontra obstáculos com base nas leituras do sensor.

    Args:
        pose (tuple): Posição (x, y) do robô.
        ranges (list of float): Lista de distâncias medidas pelo sensor.
        sensor_angle_min (float): Ângulo mínimo do sensor (rad).
        sensor_angle_max (float): Ângulo máximo do sensor (rad).
        max_distance (float): Distância máxima do sensor.

    Returns:
        list of tuple: Lista de obstáculos (x, y) em coordenadas globais.
    """
    if ranges is None:
        print("Range is None!")
        return []
    x_robot, y_robot = pose
    num_readings = len(ranges)
    obstacles = []

    # Calcula o incremento angular entre as leituras do sensor
    angle_increment = (sensor_angle_max - sensor_angle_min) / (num_readings - 1)

    for i, distance in enumerate(ranges):
        if distance < max_distance:
            # Ângulo relativo ao robô + orientação global do robô (theta)
            angle = sensor_angle_min + i * angle_increment + theta
            delta_x = distance * np.cos(angle)
            delta_y = distance * np.sin(angle)
            obstacle_x = x_robot + delta_x
            obstacle_y = y_robot + delta_y
            obstacles.append((obstacle_x, obstacle_y))

    return obstacles

def potential_field(goal=(0,0), K_att=1.0, K_rep=1.0, K_omega=1.0, epsilon_0=1.0, v_max=1.0, omega_max=np.pi/4, tol=0.1):
    global emergency_stop
    goal_x, goal_y = goal
    while not rospy.is_shutdown():
        # Verifica se o botão de emergência foi pressionado
        if emergency_stop:
            print("EMERGÊNCIA: Robô parado!")
            # Para o robô
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub.publish(twist_msg)
            return False  # Retorna False para indicar que foi interrompido
        
        x_R, y_R = get_pose()
        theta_R = orientation_bot  # Orientação do robô no intervalo [0, 2π]
        obstacles = find_obstacles(pose=get_pose(), theta=theta_R, ranges=sensor_ranges, sensor_angle_min=sensor_angle_min, sensor_angle_max=sensor_angle_max, max_distance=epsilon_0)
        # obstacles = []
        rho = np.sqrt((goal_x - x_R)**2 + (goal_y - y_R)**2)
        rospy.loginfo(f"Distância ao objetivo: {rho:.3f}")

        if rho <= tol:
            print(f"Objetivo alcançado: ({goal_x}, {goal_y})")
            break

        # Força atrativa
        F_att = K_att * np.array([goal_x - x_R, goal_y - y_R])

        # Força repulsiva
        F_rep = np.array([0.0, 0.0])
        for obstacle in obstacles:
            x_0, y_0 = obstacle
            epsilon_i = np.sqrt((x_0 - x_R)**2 + (y_0 - y_R)**2)
            if epsilon_i < epsilon_0:
                F_rep += K_rep * (1/epsilon_i**3) * (1/epsilon_0 - 1/epsilon_i) * np.array([x_0 - x_R, y_0 - y_R])

        # Força total
        F_tot = F_att + F_rep

        # Velocidade linear e angular
        v = min(np.linalg.norm(F_tot), v_max)
        phi = np.arctan2(F_tot[1], F_tot[0])  # Ângulo alvo (direção da força total)

        # Ajuste da diferença angular para o intervalo [-π, π]
        delta = phi - theta_R  # Diferença bruta
        delta_ajustado = np.mod(delta + np.pi, 2 * np.pi) - np.pi  # Força o intervalo [-π, π]

        # Velocidade angular (contínua)
        omega = K_omega * delta_ajustado
        omega = np.sign(omega) * min(abs(omega), omega_max)  # Limita a velocidade angular

        # Publicar para o robô P3dx
        #global pub
        twist_msg = Twist()  # Criando uma instância de Twist
        twist_msg.linear.x = v       # Velocidade linear
        twist_msg.angular.z = omega  # Velocidade angular

        # Publicar a mensagem
        pub.publish(twist_msg)
        
        rospy.sleep(0.03)
    
    return True  # Retorna True se chegou ao objetivo

def keyboard_listener():
    """Thread para detectar teclas pressionadas"""
    global emergency_stop
    
    # Configura o terminal para captura de teclas
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(sys.stdin.fileno())
        
        while not rospy.is_shutdown():
            # Verifica se há tecla disponível
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == ' ':  # SPACE
                    emergency_stop = True
                    print("\nBOTÃO DE EMERGÊNCIA ATIVADO!")
                elif key == '\x03':  # Ctrl+C
                    break
    except:
        pass
    finally:
        # Restaura configurações do terminal
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def odometry_callback(msg):
    global pose_bot, orientation_bot

    # Extrai a orientação como quaternion
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    # Converte quaternion para ângulos de Euler
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])

    position = msg.pose.pose.position
    pose_bot = (position.x, position.y)
    orientation_bot = yaw

    # # Exibe o ângulo yaw (em radianos e graus)
    # rospy.loginfo(f"Yaw (rad): {yaw:.4f} | Yaw (graus): {yaw * 180 / 3.14159:.2f}")

def laser_callback(msg):
    global sensor_ranges
    sensor_ranges = msg.ranges

def navigate_waypoints():
    """Função principal para navegar pelos waypoints"""
    global emergency_stop
    
    print("=== NAVEGAÇÃO POR WAYPOINTS ===")
    print(f"Waypoints definidos: {WAYPOINTS}")
    print("Pressione SPACE a qualquer momento para parada de emergência")
    print("Pressione Ctrl+C para sair")
    print("=" * 40)
    
    tol = 0.1
    
    try:
        for i, waypoint in enumerate(WAYPOINTS):
            if rospy.is_shutdown():
                break
                
            print(f"\n--- Navegando para waypoint {i+1}/{len(WAYPOINTS)}: {waypoint} ---")
            
            # Reseta flag de emergência para cada waypoint
            emergency_stop = False
            
            # Navega para o waypoint
            success = potential_field(
                goal=waypoint, 
                K_att=5, 
                K_rep=0.05, 
                epsilon_0=2.0, 
                v_max=0.3, 
                omega_max=2*np.pi, 
                tol=tol
            )
            
            if not success:
                print("Navegação interrompida por emergência!")
                break
                
            # Pequena pausa entre waypoints
            rospy.sleep(1.0)
            
        print("\n=== Navegação concluída! ===")
        
    except KeyboardInterrupt:
        print("\nNavegação interrompida pelo usuário")
    
    finally:
        # Para o robô
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)

def main():
    global pub
    rospy.init_node('p3dx_multi_waypoints', anonymous=True)

    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    sub_odometria = rospy.Subscriber("/RosAria/odom", Odometry, odometry_callback)
    sub_sensor    = rospy.Subscriber("/RosAria/laser/scan", LaserScan, laser_callback)

    # Inicia thread para detectar teclas
    keyboard_thread = threading.Thread(target=keyboard_listener)
    keyboard_thread.daemon = True
    keyboard_thread.start()

    # Aguarda um pouco para inicializar
    rospy.sleep(2.0)
    
    # Inicia a navegação por waypoints
    navigate_waypoints()

if __name__ == '__main__':
    main() 
