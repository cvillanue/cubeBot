# cubeBot: ROS 2 + Gazebo Classic 🤖

![cubeBot2](https://github.com/user-attachments/assets/60069ae2-3253-436c-aa57-dbe5bb9059ae)![cubeBot1](https://github.com/user-attachments/assets/89a32c2b-d9b8-4e7d-a4ea-84451d9015a9)




A tiny **cubeBot**  built for ROS 2 (Humble / Jazzy) with a differential-drive base and a 2D LiDAR - hoping to make this work well for reinforcement-learning experiments and robotics demos. If there's anything wonky please let me know ...it took me over a month to get this to work, im tired lol

It includes:
- Gazebo Classic world & spawn launch  
- `/cmd_vel` → differential-drive control plugin  
- `/scan` from simulated LiDAR (`sensor_msgs/LaserScan`)  
- Basic obstacle-avoidance node  
- Random-policy node + scan logger for quick RL prototyping  

---

##  Quick Start

```bash
# 1️ Create a workspace and clone the repo
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/cubebot.git
cd ~/ros2_ws

# 2️Install dependencies (Ubuntu 22.04 + ROS 2 Humble)
sudo apt update
sudo apt install -y   ros-humble-gazebo-ros-pkgs   ros-humble-xacro   ros-humble-robot-state-publisher   ros-humble-joint-state-publisher-gui

# 3️Build & source
colcon build --symlink-install
source install/setup.bash

# 4️ Run the simulation 
ros2 launch cubebot cubebot_sim.launch.py
```

**Tips**
```bash
# If Gazebo says "Address already in use", free the port:
pkill -9 gazebo gzserver gzclient 2>/dev/null

# Switch to a random driving policy instead of obstacle avoidance:
ros2 run cubebot random_policy
```

---

## Topics
| Topic | Type | Description |
|-------|------|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | velocity command |
| `/odom` | `nav_msgs/Odometry` | published by diff-drive plugin |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR range data |

---

## TF Frames
```
odom → base_link → lidar_link
```

---

##  Reinforcement Learning Notes
- **Observation (example):** downsampled LiDAR scan (≈ 36 beams) + linear/angular speeds + previous action  
- **Action:** `[linear_x, angular_z]`  
- **Reward suggestion:**  
  `+v_forward * dt` − `turn_penalty * |ω|` − `collision_penalty` (if min range < threshold)  
- For Gymnasium integration: wrap the environment to publish on `/cmd_vel` and read `/scan` + `/odom` or TF for pose.

---

## Parameters (obstacle_avoider)
| Parameter | Default | Description |
|------------|----------|-------------|
| `linear_speed` | 0.4 | forward velocity |
| `turn_speed` | 0.8 | angular speed when turning |
| `min_range` | 0.6 | minimum LiDAR range before turning |
| `fov_deg` | 60.0 | field of view used for obstacle check |

---

## Test Controls
```bash
# Move forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn in place
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

---

## Notes
- Requires **Gazebo Classic**, not Ignition Gazebo / Fortress.  
- Works with both **ROS 2 Humble** and **ROS 2 Jazzy** (update the `ros-*` package names accordingly).  
- For tele-operation:
  ```bash
  sudo apt install ros-humble-teleop-twist-keyboard
  ros2 run teleop_twist_keyboard teleop_twist-keyboard
  ```

---

## ❤️ Acknowledgments
