In order to run the demo, follow these steps:


1. Im a first terminal, run
```
export TURTLEBOT3_MODEL=waffle
ros2 launch cslam_experiments gazebo.launch.py
```

2. In a second termminal, run
```
export NB_ROBOTS=2
export TURTLEBOT3_MODEL=waffle
ros2 launch cslam_experiments turtlebot3_demo.launch.py x0:=-2 y0:=0.5 x1:=-6 y1:=-0.5
```

3. In two seperate terminals, run
```
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/turtlebot3_0/cmd_vel --ros-args -r __node:=teleop_r0
```
and
```
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/turtlebot3_1/cmd_vel --ros-args -r __node:=teleop_r1
```

Robot model and Map model can be found in (models)[cslam_experiments/models] and (worlds)[cslam_experiments/worlds].