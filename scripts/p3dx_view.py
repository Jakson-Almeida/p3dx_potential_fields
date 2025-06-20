#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time  # Added for time-based control

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from collections import deque
import threading

# User-configurable settings
SHOW_OBSTACLES = True
SHOW_POSE = True
SHOW_TRAJECTORY = True
MAX_OBSTACLES = 10000
PLOT_INTERVAL_MS = 100  # 10 FPS (100ms between updates)
K_ZOOM = 1.0  # Initial zoom factor
EPSILON_0 = 2.0  # Obstacle detection range

# Thread-safe buffers
obstacle_buffer = deque(maxlen=MAX_OBSTACLES)
robot_pose_buffer = deque(maxlen=MAX_OBSTACLES)
trajectory_buffer = deque(maxlen=MAX_OBSTACLES)
buffer_lock = threading.Lock()

# Time control variables
last_obstacle_update = 0
last_plot_update = 0

class VisualizationHandler:
    def __init__(self, root):
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.setup_plot()

        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
    
    def on_scroll(self, event):
        global K_ZOOM
        if event.button == 'up':
            K_ZOOM *= 1.1
        elif event.button == 'down':
            K_ZOOM /= 1.1

        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        x_range = (xlim[1] - xlim[0]) / 2
        y_range = (ylim[1] - ylim[0]) / 2

        self.ax.set_xlim([x_center - x_range / K_ZOOM, x_center + x_range / K_ZOOM])
        self.ax.set_ylim([y_center - y_range / K_ZOOM, y_center + y_range / K_ZOOM])
    
    def update_plot(self):
        global K_ZOOM, last_plot_update
        current_time = time.time() * 1000  # Current time in milliseconds
        
        # Only update if PLOT_INTERVAL_MS has passed
        if current_time - last_plot_update >= PLOT_INTERVAL_MS:
            last_plot_update = current_time
            
            with buffer_lock:
                obstacles = list(obstacle_buffer)
                poses = list(robot_pose_buffer)
                trajectories = list(trajectory_buffer)

            self.ax.clear()
            self.ax.grid(True)

            if K_ZOOM == 1.0:
                self.ax.set_ylim(-5.5, 5.5)
                self.ax.set_xlim(-1, 10)

            if SHOW_OBSTACLES and obstacles:
                x_obs, y_obs = zip(*obstacles)
                self.ax.scatter(x_obs, y_obs, c='red', s=10, label='Obstacles', alpha=0.5)

            if SHOW_TRAJECTORY and trajectories:
                x_traj, y_traj = zip(*trajectories)
                self.ax.plot(x_traj, y_traj, 'g-', label='Trajectory', linewidth=2)

            if SHOW_POSE and poses:
                for x, y, theta in poses[-1:]:
                    dx = 0.1 * np.cos(theta)
                    dy = 0.1 * np.sin(theta)
                    self.ax.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc='blue')
                    self.ax.scatter(x, y, c='blue', s=50, label='Robot Pose')

            self.ax.legend()
            self.canvas.draw()

    def setup_plot(self):
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('P3DX Robot Visualization')
        self.ax.grid(True)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlim(0, 10)

    def on_key_press(self, event):
        global K_ZOOM
        if event.key == ' ':
            with buffer_lock:
                obstacle_buffer.clear()
                robot_pose_buffer.clear()
                trajectory_buffer.clear()
            K_ZOOM = 1.0
            self.ax.set_ylim(-5.5, 5.5)
            self.ax.set_xlim(-1, 10)
            self.canvas.draw()

def main_loop(root, vis):
    def update():
        vis.update_plot()
        root.after(PLOT_INTERVAL_MS, update)

    root.after(PLOT_INTERVAL_MS, update)

def view():
    root = tk.Tk()
    root.title("P3DX Robot Visualization")
    vis = VisualizationHandler(root)
    main_loop(root, vis)
    root.mainloop()

# Robot state variables
pose_bot = (0, 0)
orientation_bot = 0
sensor_ranges = None
sensor_angle_min = -2.09
sensor_angle_max = 2.09

def find_obstacles(pose=(0, 0), theta=0.0, ranges=None):
    """Find obstacles within EPSILON_0 range"""
    if ranges is None:
        return []
        
    x_robot, y_robot = pose
    num_readings = len(ranges)
    obstacles = []
    angle_increment = (sensor_angle_max - sensor_angle_min) / (num_readings - 1)

    for i, distance in enumerate(ranges):
        if distance < EPSILON_0:
            angle = sensor_angle_min + i * angle_increment + theta
            obstacle_x = x_robot + distance * np.cos(angle)
            obstacle_y = y_robot + distance * np.sin(angle)
            obstacles.append((obstacle_x, obstacle_y))

    return obstacles

def odometry_callback(msg):
    global pose_bot, orientation_bot
    
    # Update pose
    orientation = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    position = msg.pose.pose.position
    pose_bot = (position.x, position.y)
    orientation_bot = yaw

    # Always update pose and trajectory
    with buffer_lock:
        robot_pose_buffer.append((position.x, position.y, yaw))
        trajectory_buffer.append((position.x, position.y))

def laser_callback(msg):
    global sensor_ranges, sensor_angle_min, sensor_angle_max, last_obstacle_update
    current_time = time.time() * 1000  # Current time in milliseconds
    
    # Only process obstacles if PLOT_INTERVAL_MS has passed
    if current_time - last_obstacle_update >= PLOT_INTERVAL_MS:
        last_obstacle_update = current_time
        
        sensor_ranges = msg.ranges
        sensor_angle_min = msg.angle_min
        sensor_angle_max = msg.angle_max
        
        obstacles = find_obstacles(pose=pose_bot, 
                                 theta=orientation_bot, 
                                 ranges=sensor_ranges)
        
        # Only add NUM obstacles to prevent overcrowding
        NUM = 20
        if len(obstacles) >= NUM:
            step = len(obstacles) / NUM
            sampled_obstacles = [obstacles[int(i * step)] for i in range(NUM)]
            with buffer_lock:
                obstacle_buffer.extend(sampled_obstacles)
        else:
            with buffer_lock:
                obstacle_buffer.extend(obstacles)

def main():
    rospy.init_node('p3dx_visualization', anonymous=True)
    rospy.Subscriber("/RosAria/odom", Odometry, odometry_callback)
    rospy.Subscriber("/RosAria/laser/scan", LaserScan, laser_callback)
    view()

if __name__ == '__main__':
    main()
