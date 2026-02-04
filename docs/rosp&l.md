# ROS 2 Publisher and Subscriber Project

## Description

This project demonstrates a basic ROS 2 publisher–subscriber architecture.
A fixed number is published periodically on a topic, received by another node,
accumulated, and then republished. The project also includes a simple string
listener example.

---

## Nodes Overview

- **publisher**: Publishes a fixed integer value to `/number`
 - **listener**: Subscribes to a string topic named `speaker`

---

## Source Code

### number_publisher.py

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class MyNode(Node):
    def _init_(self):
        super()._init_("py_test")
        self.counter = 0
        self.get_logger().info("ok ok")
        self.create_timer(1.0, self.timer_callback)  # Create a timer that calls timer_callback every 1 second
        self.publisher_ = self.create_publisher(String,'speaker',10)
    def timer_callback(self):
     self.get_logger().info("Timer callback called")
     msg = String()
     msg.data = 'hello hello : %d' % self.counter
     self.publisher_.publish(msg) 

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    my_node = MyNode()  # Create an instance of the MyNode class
    rclpy.spin(my_node)  # Keep the node running
    rclpy.shutdown()      # Shutdown rclpy

if _name_ == "_main_":
   main()
```



### listener.py

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__("listener")

        
        self.create_subscription(
            String,
            "speaker",
            self.callback,
            10
        )

    
        self.publisher_ = self.create_publisher(String, "listener_response", 10)

    def callback(self, msg):
        
        self.get_logger().info(f"I Heard: {msg.data}")

       
        response = String()
        response.data = f"Received: {msg.data}"
        
        
        self.publisher_.publish(response)
        self.get_logger().info(f"Sent Response: {response.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

---

## Execution

```bash
ros2 run brians_pkg r2d2
ros2 run brians_pkg listener
```

---

## Terminal Output

![terminal Screenshot](recursos/imgs/resultado1.png)

## Results

Here is a screenshot of the terminal output showing the publisher and listener in action:
![Diagram results](recursos/imgs/ss.png)
---

## Conclusion

This project validates the use of ROS 2 nodes for inter-process communication
using topics. The publisher–subscriber pattern allows data to be shared and
processed in real time, serving as a foundational concept for distributed
robotic systems and future mechatronics applications.
