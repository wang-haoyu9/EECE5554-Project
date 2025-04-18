# EECE5554-Project
Group 9 Project for EECE5554, Spring 2025.

Proposal Slides:
https://docs.google.com/presentation/d/1sEobmRi8D9CThlQII7NF8tmrV73d_tyd9iC1m3vps0g/edit?usp=sharing

Run this project:

```Shell
git clone https://github.com/wang-haoyu9/EECE5554-Project.git
cd EECE5554-Project/
colcon build
source install/setup.bash
ros2 launch pointcloud pc_launch.py ply:="EECE5554-Project/data/box1_high density.ply" rotation:="90,0,0"
```

Please replace `"EECE5554-Project/data/box1_high density.ply"` with absolute path if the node says it cannot find the ply file.

To see the robot control on the turtlesim_node, run this after the project starts:

```Shell
ros2 run turtlesim turtlesim_node --ros-args --remap /turtle1/cmd_vel:=/cmd_vel
```