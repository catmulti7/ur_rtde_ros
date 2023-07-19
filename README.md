# ur_rtde_ros2
An ur_rtde ros2 package, can be used to replace Universal_Robots_ROS2_Driver, support MoveIt2.  

# WARNING
**This repo is NEVER tested with real robot**, compare with Universal_Robots_ROS2_Driver, **this repo lacks the security aspect of ensuring safety of opreator and robot**.  
It is strongly recommanded to use this repo **only with URSim**. If you want to use it with real robot, make sure you fully understand this repo, ur_rtde, and take the risk on your own.

## Feature & Implement
The postion control interface provided by MoveIt is sequential, which means when users call `MoveGroupInterface::move` or `MoveGroupInterface::execute`, program will block until robot arm reaches the given position. When control command is given during the robot arm is moving, the arm will come to a stop, and than start to move to the second desire pose. In many situation these stoppings is not what we want.   
This problem can be conquer with velocity control or MoveIt Servo. But ur_rtde provide a simple interface `RTDEControlInterface::ServoJ` to achieve our goal. This repo use `MoveGroupInterface::plan` to generate joint trajectory, then give the trajectory direct to `RTDEControlInterface::ServoJ`. Since plan and execute works parallel, even before arm reaches desire position, trajectory to a new desire position will still be caculate and send to ur_rtde, replacing current executing trajectory.

## Dependenices  
this repo is tested under ubuntu22.04 and ROS Humble  
install moveit2  

    sudo apt install ros-humble-moveit
  
install ur_rtde  

    sudo add-apt-repository ppa:sdurobotics/ur-rtde
    sudo apt-get update
    sudo apt install librtde librtde-dev
    

## Usage
### clone the repo and compile  

    mkdir -p ur_rtde_ros2/src
    cd ur_rtde_ros2/src
    git clone git@github.com:catmulti7/ur_rtde_ros2.git
    cd ..
    colcon build --mixin release

### launch robot driver

Open a new terminal in workspace (always source the workspace) and run rtde_driver node. If your URSim's ip is not `192.168.56.101`, you need to modify `rtde_driver.cpp`

    ros2 run rtde_driver rtde_driver


### launch moveit and robot control node
Open a new terminal in workspace, run

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_fake_hardware:=false
This command will open a Rviz for visualize. If you don't want it, change `launch_rviz:=true` to `launch_rviz:=false`

### run command input noder
Open a new terminal in workspace, run

    ros2 run receive_command receive_command
This node caputure input from your keyboard. In this terminal you can use W/A/S/D/Q/E to control the robot

key | Axis | Movement(m)
------------ | ------------- | ----------
W | x |+0.02
S | x |-0.02
A | y |+0.02
D | y |-0.02
Q | z |+0.02
E | z |-0.02
