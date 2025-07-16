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
Sure! Here's your original list converted into clean Markdown format:

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

# Orange Cone Detection with ROS 2, Gazebo, and OpenCv
## Project Structure

ros2_ws/
├── src/
│   └── my_robot/
│       ├── launch/
│       │   └── view_robot.launch.py
│       ├── urdf/
│       │   └── four_wheel_bot.xacro
│       ├── world/
│       │   └── obstacle_world.world
│       ├── models/
│       │   └── orange_cone/...
│       └── my_robot/
│           └── cone_detector.py

## Steps to Run

1. Install ROS 2 Humble

Make sure ROS 2 Humble and Gazebo 11 are installed and sourced.

2. Clone and Build

cd ~/ros2_ws  
colcon build --symlink-install  
source install/setup.bash

3. Launch Simulation

ros2 launch my_robot view_robot.launch.py

4. Run Detection Node

ros2 run my_robot cone_detector

## Cone Model

We used a construction-style cone downloaded from GitHub.  
Placed in: `~/.gazebo/models/orange_cone/`

We updated `model.sdf` to use the mesh and scaled it:

<mesh>
  <uri>model://orange_cone/meshes/construction_cone.dae</uri>
  <scale>10 10 10</scale>
</mesh>

## Detection Logic

The `cone_detector.py` node:
- Subscribes to `/camera/image_raw`
- Converts ROS image to OpenCV using cv_bridge
- Filters orange color in HSV
- Draws bounding box and displays the image

## Common Issues and Fixes

Problem: Gazebo fails with "Address already in use"  
Fix: kill -9 <gzserver PID>

Problem: Cone not visible  
Fix: Increase <scale> and adjust <pose> in world file

Problem: No mesh specified  
Fix: Check <geometry><mesh><uri> 

Problem: GUI issues inside Docker container  
Fix: Export DISPLAY, LIBGL_ALWAYS_SOFTWARE, and XDG_RUNTIME_DIR


## Final Result

- The robot and cone are visible in Gazebo
- The OpenCV window highlights the orange cone 

## Credits

Cone Model: https://github.com/osrf/gazebo_models/tree/master/construction_cone  

## Summary

- Created a robot with camera and spawned in Gazebo
- Added a construction-style cone model to the world
- Wrote and ran OpenCV node to detect orange color
- Faced errors like model not found, gzserver already running, and mesh URI issues
- Solved by killing existing gazebo instances, checking model paths, and correcting world sdf file
