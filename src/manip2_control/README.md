# manip2_control

ROS Noetic bridge for a 5-DoF Dynamixel manipulator (mixed Protocol 1.0 + 2.0).
- Accepts **degree** commands (URDF convention).
- Enforces joint limits.
- Converts to ticks using nominal mapping.
- **Initial readback** after torque ON shows current joints.
- **Periodic readback** printed and published on `/joint_states`.
- Works in WSL2.

## Install

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-robot-state-publisher ros-noetic-xacro
pip3 install -r ~/catkin_ws/src/manip2_control/requirements.txt
```

## Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch manip2_control manip2_control.launch urdf_path:=/home/aliefpal/catkin_ws/src/manip2_urdf_description/urdf/manip2_urdf.xacro port_name:=/dev/ttyS3
```

### Send a command (degrees)
```bash
rostopic pub /manip2/command_deg std_msgs/Float64MultiArray "data: [0, 20, -90, 20, 45]" -1
```

### Go HOME
```bash
rosservice call /manip2/go_home
```

### View joint states
```bash
rostopic echo /joint_states
```

## Notes (WSL2)

- COM3 often maps to `/dev/ttyS3`. If using usbipd, it might be `/dev/ttyUSB0`. Pass `port_name:=...` accordingly.
- If permission issues:
  ```bash
  sudo usermod -a -G dialout $USER
  sudo chmod a+rw /dev/ttyS3
  ```
- `joint_states` uses names `joint1..joint5` (match your URDF).
