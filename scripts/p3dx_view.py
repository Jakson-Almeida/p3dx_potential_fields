#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations  # Para converter quaternion → Euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from collections import deque
import threading
import time
import json

# Configurações do usuário (podem ser modificadas)
SHOW_OBSTACLES = True
SHOW_POSE = True
SHOW_TRAJECTORY = True
MAX_OBSTACLES = 10000
PLOT_INTERVAL_MS = 100  # 10 FPS
K_ZOOM = 1.0  # Fator de zoom inicial

# Buffer de obstáculos (thread-safe)
obstacle_buffer = deque(maxlen=MAX_OBSTACLES)
robot_pose_buffer = deque(maxlen=MAX_OBSTACLES)
trajectory_buffer = deque(maxlen=MAX_OBSTACLES)
buffer_lock = threading.Lock()

class VisualizationHandler:
    def __init__(self, root):
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.animation = None
        self.setup_plot()

        # Conectar eventos de teclado
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
    
    def on_scroll(self, event):
        global K_ZOOM

        # Determina a direção do scroll (up = zoom in, down = zoom out)
        if event.button == 'up':
            K_ZOOM *= 1.1  # Aumenta o zoom em 10%
        elif event.button == 'down':
            K_ZOOM /= 1.1  # Diminui o zoom em 10%

        # Ajusta os limites dos eixos com base no fator de zoom
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        # Calcula novos limites
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        x_range = (xlim[1] - xlim[0]) / 2
        y_range = (ylim[1] - ylim[0]) / 2

        # Aplica o zoom
        self.ax.set_xlim([x_center - x_range / K_ZOOM, x_center + x_range / K_ZOOM])
        self.ax.set_ylim([y_center - y_range / K_ZOOM, y_center + y_range / K_ZOOM])
    
    def update_plot(self):
        global K_ZOOM
        with buffer_lock:
            obstacles = list(obstacle_buffer)
            poses = list(robot_pose_buffer)
            trajectories = list(trajectory_buffer)

        self.ax.clear()
        self.ax.grid(True)

        # Aplica os limites de zoom apenas se K_ZOOM for 1.0 (zoom padrão)
        if K_ZOOM == 1.0:
            self.ax.set_ylim(-5.5, 5.5)
            self.ax.set_xlim(-1, 10)

        if SHOW_OBSTACLES and obstacles:
            x_obs, y_obs = zip(*obstacles)
            self.ax.scatter(x_obs, y_obs, c='red', s=10, label='Obstáculos', alpha=0.5)

        if SHOW_TRAJECTORY and trajectories:
            x_traj, y_traj = zip(*trajectories)
            self.ax.plot(x_traj, y_traj, 'g-', label='Trajetória', linewidth=2)

        if SHOW_POSE and poses:
            # x_pose, y_pose, theta_pose = zip(*poses)
            # self.ax.scatter(x_pose, y_pose, c='blue', s=50, label='Pose do Robô')
            # Plotar orientação
            for x, y, theta in poses[-1:]:  # Mostra apenas a última pose
                dx = 0.1 * np.cos(theta)
                dy = 0.1 * np.sin(theta)
                self.ax.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc='black')

        self.ax.legend()
        self.canvas.draw()

    def setup_plot(self):
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Visualização em Tempo Real')
        self.ax.grid(True)
        self.ax.set_ylim(-5, 5)  # Limites iniciais do eixo Y
        self.ax.set_xlim(0, 10)  # Limites iniciais do eixo X

    def on_key_press(self, event):
        global K_ZOOM
        # Limpa os buffers quando a tecla espaço é pressionada
        if event.key == ' ':
            with buffer_lock:
                obstacle_buffer.clear()
                robot_pose_buffer.clear()
                trajectory_buffer.clear()
            K_ZOOM = 1.0  # Reseta o zoom
            self.ax.set_ylim(-5.5, 5.5)
            self.ax.set_xlim(-1, 10)
            self.canvas.draw()
            # print("Buffers zerados!")

def save_data():
    while True:
        time.sleep(PLOT_INTERVAL_MS / 1000)  # Sincronizado com o frame rate
        with buffer_lock:
            data = {
                "obstacles": list(obstacle_buffer),
                "poses": list(robot_pose_buffer),
                "trajectories": list(trajectory_buffer)
            }
        with open("robot_data.json", "w") as f:
            json.dump(data, f)

def main_loop(root, vis):
    # Thread para salvar dados
    save_thread = threading.Thread(target=save_data)
    save_thread.daemon = True
    save_thread.start()

    # Atualiza o gráfico periodicamente
    def update():
        vis.update_plot()
        root.after(PLOT_INTERVAL_MS, update)

    root.after(PLOT_INTERVAL_MS, update)

def view():
    root = tk.Tk()
    vis = VisualizationHandler(root)
    main_loop(root, vis)
    root.mainloop()

pub = None
pose_bot = (0, 0)
orientation_bot = 0
sensor_ranges = None
sensor_angle_min = -2.09
sensor_angle_max =  2.09

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
    goal_x, goal_y = goal
    while not rospy.is_shutdown():
        x_R, y_R = get_pose()
        theta_R = orientation_bot  # Orientação do robô no intervalo [0, 2π]
        obstacles = find_obstacles(pose=get_pose(), theta=theta_R, ranges=sensor_ranges, sensor_angle_min=sensor_angle_min, sensor_angle_max=sensor_angle_max, max_distance=epsilon_0)
        # obstacles = []
        robot_pose_buffer.append((x_R, y_R, theta_R))
        trajectory_buffer.append((x_R, y_R))
        NUM = 20
        if len(obstacles) >= NUM:
            # Calcula o passo para selecionar os pontos uniformemente
            step = len(obstacles) / NUM
            for i_num in range(NUM):
                # Garante que o índice seja inteiro
                index = int(i_num * step)
                obstacle_buffer.append(obstacles[index])
        else:
            for ind, obs in enumerate(obstacles):
                obstacle_buffer.append(obs)
        rho = np.sqrt((goal_x - x_R)**2 + (goal_y - y_R)**2)
        rospy.loginfo(rho)

        if rho <= tol:
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
        global pub
        twist_msg = Twist()  # Criando uma instância de Twist
        twist_msg.linear.x = v       # Velocidade linear
        twist_msg.angular.z = omega  # Velocidade angular

        # Publicar a mensagem
        pub.publish(twist_msg)
        
        rospy.sleep(0.03)

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

def menu():
    global pub
    tol = 0.1
    # Loop principal integrado
    try:
        while not rospy.is_shutdown():
            print("Ponto Objetivo do robô:")
            try:
                x = float(input("Posição em x: "))
                y = float(input("Posição em y: "))
            except:
                break
            # x = np.random.uniform(-10,10)
            # y = np.random.uniform(-10,10)
            # print(f"Sorteio x={x}")
            # print(f"Sorteio y={x}")
            potential_field(goal=(x,y), K_att=5, K_rep=0.05, epsilon_0=2.0, v_max=0.3, omega_max=np.pi/2, tol=tol)
            twist_msg = Twist()  # Criando uma instância de Twist
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0

            # Publish the message
            pub.publish(twist_msg)
    except:
        pass

def main():
    global pub
    rospy.init_node('p3dx_potential_field', anonymous=True)

    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    sub_odometria = rospy.Subscriber("/RosAria/odom", Odometry, odometry_callback)
    sub_sensor    = rospy.Subscriber("/RosAria/laser/scan", LaserScan, laser_callback)

    menu_thread = threading.Thread(target=menu)
    menu_thread.daemon = True
    menu_thread.start()

    view()


if __name__ == '__main__':
    main()
