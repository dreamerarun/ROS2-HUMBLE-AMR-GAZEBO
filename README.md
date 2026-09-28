# my_robot_description

A differential-drive robot built from scratch in **ROS2 Humble** + **Gazebo
Harmonic (gz sim)**, with lidar + camera sensors, SLAM mapping, and
autonomous Nav2 navigation (including multi-waypoint routes).

Built as a step-by-step learning project — the full write-up of every
design decision and every bug encountered along the way is in
[`docs/BUILD_GUIDE.md`](docs/BUILD_GUIDE.md).

## Features

- Custom diff-drive robot defined in xacro (chassis, 2 driven wheels +
  caster, lidar, camera)
- Gazebo Harmonic simulation with a custom world (obstacles + perimeter
  walls)
- ROS2 ↔ Gazebo bridging via `ros_gz_bridge` (cmd_vel, odom, scan, camera,
  tf, joint_states)
- Teleop keyboard control
- SLAM mapping via `slam_toolbox`
- Autonomous navigation via Nav2 (AMCL localization, costmaps, planner,
  controller, recovery behaviors)
- Multi-point waypoint navigation (RViz panel + `nav2_simple_commander`
  script)

## Package layout

```
my_robot_description/
├── urdf/
│   └── my_robot.urdf.xacro       # robot description
├── launch/
│   ├── gazebo.launch.py          # spawn robot + world in gz sim
│   ├── rviz.launch.py            # RViz with pre-configured displays
│   ├── slam.launch.py            # slam_toolbox mapping
│   └── navigation.launch.py      # Nav2 bringup
├── worlds/
│   └── my_world.sdf              # obstacles + perimeter walls
├── config/
│   ├── slam_toolbox_params.yaml
│   └── nav2_params.yaml
├── rviz/
│   └── robot.rviz                # saved display config
├── maps/
│   └── my_map.yaml / .pgm        # saved SLAM map
├── waypoint_nav.py               # scripted multi-waypoint navigation
└── docs/
    └── BUILD_GUIDE.md            # full write-up: design + every bug fixed
```

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Harmonic (`gz sim`) + `ros_gz_sim`, `ros_gz_bridge`
- `ros-humble-slam-toolbox`
- `ros-humble-navigation2`, `ros-humble-nav2-bringup`
- `ros-humble-nav2-simple-commander` (for the waypoint script)
- `ros-humble-teleop-twist-keyboard`

## Setup

```bash
cd ~/ros_ws/src
git clone <this-repo-url> my_robot_description
cd ~/ros_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Usage

**1. Launch the simulation:**
```bash
ros2 launch my_robot_description gazebo.launch.py
```

**2. Drive it manually:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**3. Visualize (lidar, camera, TF, odometry):**
```bash
ros2 launch my_robot_description rviz.launch.py
```

**4. Build a map (SLAM):**
```bash
ros2 launch my_robot_description slam.launch.py
# drive around with teleop, then:
ros2 run nav2_map_server map_saver_cli -f ~/ros_ws/src/my_robot_description/maps/my_map
```

**5. Autonomous navigation:**
```bash
ros2 launch my_robot_description navigation.launch.py
```
In RViz: **2D Pose Estimate** to localize, then **2D Goal Pose** / **Nav2
Goal** to send a destination.

**6. Waypoint navigation:**
```bash
python3 waypoint_nav.py
```
Or use the **Waypoint / Nav Through Poses Mode** button in RViz's
Navigation 2 panel.

## Notes

Written and debugged interactively — see
[`docs/BUILD_GUIDE.md`](docs/BUILD_GUIDE.md) for the reasoning behind every
design choice and a catalog of the ~17 configuration bugs hit (and fixed)
along the way: xacro quirks, URDF→SDF link lumping, `ros_gz_bridge` frame
sanitization, RViz QoS/display gotchas, and several Nav2 pluginlib naming
mismatches.

## License

MIT 
