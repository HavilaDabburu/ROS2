# ROS2
# ROS 2 Learning Progress

## ✅ Topics I Learned

- **Install Docker and its setup**
- **Install and run ROS 2 Humble in Docker container**
- **Setting up ROS 2 environment**
- **Using Git in Docker**
- **Saving containers in Docker**
- **How to install `turtlesim` and `rqt`**
- **Understanding nodes**
- **Understanding services**
- **Understanding topics**
- **Using `colcon` to build packages**
- **Creating a workspace**
- **Writing a simple publisher and subscriber (Python)**

# MY HANDWRITTEN NOTES
- https://drive.google.com/file/d/15DDReQzUQ5XeWPcQao1XMZ_7PkywSPPK/view?usp=drivesdk
# ROS 2 Humble Notes (Using Docker)

## 1. Install Docker
Install from: [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/)

Verify installation:
```bash
docker --version
```

---

## 2. Pull and Run ROS 2 Humble in Docker
```bash
docker pull osrf/ros:humble-desktop
docker run -it --name ros2_container osrf/ros:humble-desktop
```

To reuse the container:
```bash
docker start -ai ros2_container
```

---

## 3. Set Up ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## 4. Install Tools (Turtlesim & RQT)
```bash
apt update
apt install -y ros-humble-turtlesim ros-humble-rqt
```

Run:
```bash
ros2 run turtlesim turtlesim_node
ros2 run rqt_gui rqt_gui
```

---

## 5. Nodes, Topics, and Services (Basics)
```bash
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/pose
ros2 service list
```

---

## 6. Install Colcon
```bash
apt install -y python3-colcon-common-extensions
```

---

## 7. Create and Build a ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 8. Create Python Package with Publisher & Subscriber

Create package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

### Folder structure:
```
ros2_ws/
└── src/
    └── py_pubsub/
        ├── py_pubsub/
        │   ├── publisher_member_function.py
        │   └── subscriber_member_function.py
        ├── setup.py
        ├── package.xml
        └── resource/
```

### Publisher: `publisher_member_function.py`
```python
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
```

### Subscriber: `subscriber_member_function.py`
```python
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
```

### Build and Source
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run the Nodes
```bash
ros2 run py_pubsub publisher_member_function
ros2 run py_pubsub subscriber_member_function
```

---

## Sources
- [ROS 2 Humble Official Website](https://docs.ros.org/en/humble/index.html)
- ROS 2 Docker Tutorials
