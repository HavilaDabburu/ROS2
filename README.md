

https://github.com/user-attachments/assets/6e02ab43-b88a-4ada-b1a6-0badf35dd2ab

# ROS2
# ROS 2 Learning Progress

## ✅ Topics I Learned

- *Install Docker and its setup*
- *Install and run ROS 2 Humble in Docker container*
- *Setting up ROS 2 environment*
- *Using Git in Docker*
- *Saving containers in Docker*
- *How to install turtlesim and rqt*
- *Understanding nodes*
- *Understanding services*
- *Understanding topics*
- *Using colcon to build packages*
- *Creating a workspace*
- *Writing a simple publisher and subscriber (Python)*

# MY HANDWRITTEN NOTES
- https://drive.google.com/file/d/15DDReQzUQ5XeWPcQao1XMZ_7PkywSPPK/view?usp=drivesdk
# ROS 2 Humble Notes (Using Docker)

## 1. Install Docker
Install from: [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/)

Verify installation:
bash
docker --version


---

## 2. Pull and Run ROS 2 Humble in Docker
bash
docker pull osrf/ros:humble-desktop
docker run -it --name ros2_container osrf/ros:humble-desktop


To reuse the container:
bash
docker start -ai ros2_container


---

## 3. Set Up ROS 2 Environment
bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


---

## 4. Install Tools (Turtlesim & RQT)
bash
apt update
apt install -y ros-humble-turtlesim ros-humble-rqt


Run:
bash
ros2 run turtlesim turtlesim_node
ros2 run rqt_gui rqt_gui


---

## 5. Nodes, Topics, and Services (Basics)
bash
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/pose
ros2 service list


---

## 6. Install Colcon
bash
apt install -y python3-colcon-common-extensions


---

## 7. Create and Build a ROS 2 Workspace
bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash


---

## 8. Create Python Package with Publisher & Subscriber

Create package:
bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub


### Folder structure:

ros2_ws/
└── src/
    └── py_pubsub/
        ├── py_pubsub/
        │   ├── publisher_member_function.py
        │   └── subscriber_member_function.py
        ├── setup.py
        ├── package.xml
        └── resource/


### Publisher: publisher_member_function.py
python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from publisher'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


### Subscriber: subscriber_member_function.py
python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


### Build and Source
bash
cd ~/ros2_ws
colcon build
source install/setup.bash


### Run the Nodes
bash
ros2 run py_pubsub publisher_member_function
ros2 run py_pubsub subscriber_member_function


---

## Sources
- [ROS 2 Humble Official Website](https://docs.ros.org/en/humble/index.html)
- ROS 2 Docker Tutorials
# week 2
# What I Learnt

1. *DDS and peer-to-peer in ROS 2*
   - Understood how ROS 2 uses DDS for peer-to-peer communication.

2. *Why ROS 2 moved away from ROS 1's master*
   - Learned the limitations of the centralized ROS 1 master and the benefits of ROS 2's decentralized architecture.

3. *Skimmed ROS 2 discovery mechanisms*
   - Briefly went through concepts like UDP multicast, discovery server, static discovery, etc.

Let me know if you want this turned into a more detailed Markdown journal or study notes!


##  What is DDS?

*DDS* stands for *Data Distribution Service*.  
It is a *middleware protocol and API standard* 

DDS helps different software systems *communicate with each other in real-time. It is designed to be **fast, **reliable*, and work well even in systems where timing is very important — like robotics, vehicles, or medical devices.

In DDS, data is shared using a *publish/subscribe model*:
- A *publisher* sends data on a specific topic.
- A *subscriber* receives data from that topic.
- They don’t need to know about each other directly — DDS connects them automatically.

---

## What is Peer-to-Peer Communication?

In DDS communication is *peer-to-peer*.  
This means:
- There is *no central server* or master controlling the system.
- Each node (program) can *directly talk* to other nodes.
- Nodes *automatically discover* each other on the network using something called *UDP multicast*.
- After discovering, they send messages *directly* (using unicast).

So instead of sending messages through a central point, the publisher and subscriber talk *straight to each other*.

---

## Why Did ROS 2 Drop the ROS Master?

In *ROS 1, there was **ROS Master*:
- It was a central server that kept track of all publishers and subscribers.
- If the ROS Master crashed, the whole system could stop working.
- It was hard to use ROS 1 on multiple machines or in dynamic environments.

In *ROS 2, the ROS Master is **removed*.  
Instead, DDS handles everything:
- It manages discovery and communication *automatically*.
- It works well across *multiple machines* and *complex networks*.
- It is *more flexible, reliable, and scalable*.

That’s why ROS 2 switched to *DDS and peer-to-peer communication*

# Four-Wheel-Robot

 A **four-wheeled-Robot** in **ROS 2 Humble** with **LIDAR**, **camera**, **teleoperation**, and **obstacle avoidance**, all integrated into Gazebo.

---

## Step-by-Step 

### 1. Set Up ROS 2 Humble in Docker with GUI Support

```bash
xhost +local:root

docker run -it \
  --net=host \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  ros:humble bash
```

### 2. Create ROS 2 Workspace and Package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a Python ROS 2 package
ros2 pkg create --build-type ament_python my_robot

cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
```

---

## Robot Description

### 3. Create URDF

* Create `urdf/` folder inside your package:

  ```bash
  mkdir -p ~/ros2_ws/src/my_robot/urdf
  ```

* Build `four_wheel_bot.urdf`:

  * Base link
  * 4 wheels
  * Proper joints and transforms

* Visualize it:

  ```bash
  ros2 run robot_state_publisher robot_state_publisher urdf/four_wheel_bot.urdf
  ros2 run rviz2 rviz2
  ```

---

### 4. Convert to Xacro

* Create `four_wheel_bot.xacro`
* Convert and test it:

  ```bash
  xacro four_wheel_bot.xacro > test.urdf
  check_urdf test.urdf
  ```

---

### 5. Add Differential Drive Plugin for Movement

* Added the `libgazebo_ros_diff_drive.so` plugin to the robot’s base.
* Only rear wheels are controlled initially.

---

### 6. Improve Dynamics

* Added `mass` and `inertial` tags to all links.
* Fixed physical behavior in simulation.

---

### 7. Add Skid Steering

* suggested by an intern in unofficial group.
* Enabled **skid steering** so the robot could turn smoothly.
* Skid steering = both front and rear wheels rotate (but front may not drive).
* Movement was much more natural.

---

## Sensor 

### 8. Add LIDAR Sensor

* Added LIDAR link + joint.
* Used `gazebo_ros_ray_sensor` plugin.
* Verified data:

  ```bash
  ros2 topic echo /scan
  ```

---

### 9. Add Camera Sensor

* Mounted camera on a stand.
* Added `gazebo_ros_camera` plugin.
* View the feed:

  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```

##  Behavior Node

### 10. Add Stopper Node

* Subscribed to `/scan`
* Published to `/cmd_vel`
* Stops robot when obstacle is < 1m away from LIDAR i.e 0.5 away from robot front

Used with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> The node acts as a **safety filter**: it overrides teleop **only if needed**.

---

## Gazebo World

### 11. Created Custom World with Wall

* In `world/obstacle_world.world`:

```xml
<pose>0 10 0.5 0 0 0</pose>
```

* Places a wall 10 meters in front of the robot.

---

## Final Launch

### 12. Unified Everything in `view_robot.launch.py`

```bash
ros2 launch my_robot view_robot.launch.py
```

* Launches:

  * Gazebo
  * Robot description
  * Stopper node

---

## Project Structure

```
ros2_ws/
└── src/
    └── my_robot/
        ├── urdf/
        │   └── four_wheel_bot.xacro
        ├── world/
        │   └── obstacle_world.world
        ├── launch/
        │   └── view_robot.launch.py
        ├── my_robot/
        │   └── stopper_node.py
        ├── package.xml
        └── setup.py
```

##  Useful ROS 2 Commands

```bash
# Build & Source
colcon build --packages-select my_robot
source install/setup.bash

# Launch the simulation
ros2 launch my_robot view_robot.launch.py

# Visualize LIDAR
ros2 topic echo /scan

# Drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# View camera
ros2 run rqt_image_view rqt_image_view
```

## Skid Steering

Skid steering turns by changing speed between left and right wheels:

* Equal speed → straight
* One side faster → turns
* One side backward → spin in place

Used by tanks, bulldozers, and now our robot!

---

## Final Result

* Robot launches in Gazebo
* Can be driven via keyboard
* Stops automatically near wall
* Publishes `/scan` and `/camera/image_raw`
* All integrated in one launch file
