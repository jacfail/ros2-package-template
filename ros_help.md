# ROS HELP


## Installation

A ROS description can be found in this link: http://wiki.ros.org/ROS/Introduction

Instructions for installing ROS2 can be found in this link: http://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

For Turtlebot3 installation go to the following link and select Foxy: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
The link defaults to Kinetic still, so make sure to select Foxy instructions.

Various ROS2 tutorials can be found at this link: http://docs.ros.org/en/foxy/Tutorials.html

---

## Create a Workspace

Start by sourcing your ROS2 environment and your bash, then make a new workspace directory: 
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

To grab some beginner tutorials, you can clone in the following repo into the `src` folder.
```
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
```

Make sure to resolve package depnedencies as well:
```
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
```

If you have all your dependencies, it will return: 
```
All required rosdeps installed successfully
```

Build your workspace from the root with the following command: 
```
colcon build
```

---


## ROS Sourcing and Exporting

```
. install/setup.bash
source /opt/ros/foxy/setup.bash
source ~/.bashrc
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=69
```
---

## Building a ROS Workspace

Source your ROS2 environment
```
source /opt/ros/foxy/setup.bash
```

Create a directory and add a src directory inside
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```


## Test ROS Workspace (Turtlebot)

Clone in a repo like turtlebot3
```
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Build the simulation package
```
cd ~/ros2_ws && colcon build --symlink-install
```

Test with the following commands:
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Test ROS Workspace (ROS Tutorials)

To grab some beginner tutorials, you can clone in the following repo into the `src` folder.
```
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
```

Make sure to resolve package dependencies as well:
```
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
```

If you have all your dependencies, it will return: 
```
All required rosdeps installed successfully
```

Build your workspace from the root with the following command: 
```
cd ~/ros2_ws && colcon build
```


## Building a Package
Run this whenever you need to rebuild just one package in your ROS workspace
```
colcon build --packages-select my_package
```
---

## Launching the Robot (standard)
```
ros2 launch turtlebot3_bringup robot.launch.py
```
---

## Launching the Robot (w/ camera)
```
ros2 launch turtlebot3_bringup camera_robot.launch.py
```
---

## Running a Node in a Package
```
ros2 run my_package my_node
```
---

## Connecting to Robot

```
BB8 - Team 69
ssh burger@128.61.16.165
user: burger
pw: burger
```
---

## Teleoperating the Robot
```
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=69
source ~/.bashrc
ros2 run turtlebot3_teleop teleop_keyboard
```
---

## Shutdown the Robot
```
sudo shutdown now
```
---

## Communication Issue Fix

When having issues getting ros2 rostopic list outputs
Might need to do communication issue fix (netplan, daemons stuff)

https://www.theconstructsim.com/ros2-qa-218-how-to-create-conditional-publisher-in-ros2/

https://answers.ros.org/answers/383342/revisions/


It looks like the ROS 2 communications sometimes stop working for the robots so you can't see any of the nodes or topics from the robot on your laptop and vice versa. To fix this there are a few things you need to do:

1. SSH into the robot
```
ssh burger@<ip_of_robot>
```
2. Run the following commands:
```
ros2 daemon start
sudo netplan apply && sudo shutdown now
```
3. The robot should power off so wait for the green LED on the raspberry pi to turn off and then flip the power switch to off.
4. Turn the robot on again
5. SSH into the robot again
```
ssh burger@<ip_of_robot>
```
6. Run the following:
```
ros2 daemon start
```

7. You should now be able to see topics and nodes between the robot and laptop.


## Gazebo World Launch Fix

If you get an error when trying to launch a gazebo world or model, that process has died and there is an exit code 255 in it, you might have gzclient or gzserver opened. Kill those with the following commands: 
```
killall gzserver
kilall gzclient
```

---

## Random ROS Command List

### Source your bash file
```
. install/setup.bash
source /opt/ros/foxy/setup.bash
source ~/.bashrc
```

### Check Status of ROS2
```
printenv | grep -i ROS
```

### Export your ROS2 Domain ID
```
export ROS_DOMAIN_ID=69
```


### See ROS2 Nodes, Topics, Services, and Actions:
```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### Bring up Rqt
```
rqt
rqt_graph
ros2 run rqt_console rqt_console
```


### Colcon Building and Testing Commands
1. To build and test your colcon build use the following commands:
```
colcon build
colcon test
```

2. If you want to only build one package in the ROS workspace, use the following command:
```
colcon build --packages-select my_package
```

3. If you want to avoid having to rebuild every time you tweak python scripts, use the following command:
```
colcon build --symlink-install

. install/setup.bash
colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```


### Create Package in Python
```
ros2 pkg create --build-type ament_python <package_name>
```

### Run Gazebo
```
source ~/.bashrc
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Auto Drive the Robt
```
ros2 run turtlebot3_gazebo turtlebot3_drive
```

### Run Rviz
```
ros2 launch turtlebot3_bringup rviz2.launch.py
```


### Topic Commands
```
ros2 topic echo <topic_name>
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

### Service Commands
```
ros2 service type <service_name>
ros2 service find <type_name>
ros2 interface show <type_name>.srv
ros2 service call <service_name> <service_type> <arguments>
```

### Param Commands
```
ros2 param list
ros2 param get <node_name> <parameter_name>
ros2 param set <node_name> <parameter_name> <value>
ros2 param dump <node_name>
ros2 param load <node_name> <parameter_file>
```

### Start Node Using Saved Parameter Values
```
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

### Action Commands
```
ros2 action list
ros2 action list -t
ros2 action info /turtle1/rotate_absolute
ros2 interface show turtlesim/action/RotateAbsolute
ros2 action send_goal <action_name> <action_type> <values>
```

### Rosbag Commands
```
ros2 bag record <topic_name>
ros2 bag info <bag_file_name>
ros2 bag info subset
ros2 bag play subset
```
---

## ROS Definitions

### ROS Domain ID
ROS DOMAIN ID is a unique identifier where all nodes on that domain can communicate to eachother

### Nodes
Nodes have a single purpose like controlling wheels, using laser range-finder, etc.
Nodes send send and recieve data to eachother via topics, services, actions, or parameters
You can remap nodes 
Nodes can publish data to any number of topics and get subscribed to from an number of topics

### Topics
Topics are one of the main ways data is moved between nodes and use publishing and subscribing

### Services
Services are based on a call and response model
Services only provide updates when they are called while topics continuously provide updates

### Parameters
Parameters are configuration values of nodes or node settings. Each node has its own parameters

### Actions
Actions have three parts: goal, feedback, and a result
Actions are built on topics and services. Similar to services except you can cancel them while executing and provide steady feedback.
A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.


### Message
Primary method for exchanging data on specified topics between nodes

### Subscriber
A subscriber subscribes or listens to a topic and takes specific message types from it

```
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            UInt64,
            'amazing_int',
            self.magic_fun,
            10)
        self.subscription  # prevent unused variable warning

    def magic_fun(self, msg):
        self.get_logger().info('I heard: "%i"' % msg.data)
```


### Publisher
A publisher publishes or sends from a topic a specific message type

```
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Bool, 'amazing_bool', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%b"' % msg.data)
        self.i += 1
```    
        
You can have multiple subscribers with one node.
You can have multiple publishers with one node.

