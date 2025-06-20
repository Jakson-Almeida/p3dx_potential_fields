# P3DX Potential Fields Navigation

ROS package for autonomous navigation of Pioneer P3-DX robot using potential fields algorithm in Gazebo simulation.

![Potential Fields](potential_fields.gif)

## Prerequisites

- ROS Noetic (recommended)
- Ubuntu 20.04
- Gazebo (comes with ROS desktop-full installation)
- Pioneer P3DX packages:
  ```bash
  git clone https://github.com/mario-serna/pioneer_p3dx_model.git ~/catkin_ws/src/p3dx_gazebo
  ```

## Installation

1. Clone this repository to your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Jakson-Almeida/p3dx_potential_fields.git
   ```

2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Basic Simulation

Launch the Gazebo simulation with potential fields navigation:
```bash
roslaunch p3dx_potential_fields corredor_p3dx_campos_potenciais.launch
```

### Interactive Control

1. In a new terminal, start the visualization:
   ```bash
   rosrun p3dx_potential_fields p3dx_view.py
   ```

2. In the visualization window:
   - Enter goal coordinates when prompted in the terminal
   - Press SPACE to clear the visualization
   - Use mouse scroll to zoom in/out

### Keyboard Shortcuts (Visualization)
- `SPACE`: Clear all obstacles and trajectory
- `Mouse Scroll`: Zoom in/out

## Nodes

### p3dx_potential_field.py
- Implements potential field algorithm
- Subscribes to:
  - `/RosAria/odom` (robot pose)
  - `/RosAria/laser/scan` (laser data)
- Publishes to:
  - `/RosAria/cmd_vel` (velocity commands)

### p3dx_view.py
- Implements potential field algorithm
- Visualization tool showing:
  - Robot position (blue)
  - Obstacles (red)
  - Trajectory (green)
- Interactive goal input via terminal

## Parameters

You can adjust these parameters in the code:
- `K_att`: Attractive force gain
- `K_rep`: Repulsive force gain
- `epsilon_0`: Influence distance of obstacles
- `v_max`: Maximum linear velocity
- `omega_max`: Maximum angular velocity
- `tol`: Goal tolerance distance

## Customization

1. To change the world:
   - Modify `worlds/mundo_corredor.world`
   - Update the path in the launch file

2. To tune the potential fields:
   - Edit the parameters in `p3dx_potential_field.py`

