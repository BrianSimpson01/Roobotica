# Reset Counter Service Project

## Description

This project demonstrates a ROS 2 publisher–subscriber architecture with an
additional service interface. A publisher sends string messages periodically
on a topic, while a listener node receives them, counts each message, publishes
a response, and provides a service to reset the counter.

---

## Nodes Overview

- **publisher**  
  Publishes string messages to the `speaker` topic.

- **listener**  
  - Subscribes to the `speaker` topic  
  - Counts the received messages  
  - Publishes responses on `listener_response`  
  - Provides a service `reset_listener` to reset the counter  

---

## Source Codes

### listener.py

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from example_interfaces.srv import SetBool


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.counter = 0
        self.create_subscription(
            String,
            "speaker",
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(
            String,
            "listener_response",
            10
        )
        self.create_service(
            SetBool,
            "reset_listener",
            self.handle_service
        )

        self.get_logger().info("Listener with reset service ready")

    def callback(self, msg):
        self.counter += 1

        self.get_logger().info(
            f"Message #{self.counter}: {msg.data}"
        )

        response = String()
        response.data = f"Received {self.counter} messages"
        self.publisher_.publish(response)

    def handle_service(self, request, response):
      
        if request.data:
            self.counter = 0
            response.message = "Counter reset to 0"
        else:
            response.message = "Counter continues without reset"

        response.success = True
        return response


def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
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


---

## Execution

```bash
ros2 run brians_pkg r2d2
ros2 run brians_pkg listener
ros2 service call /reset_listener example_interfaces/srv/SetBool "{data: true}"
ros2 service call /reset_listener example_interfaces/srv/SetBool "{data: false}"
rqt_graph

```

---

## Terminal Output
in this photo you can see in the terminal on the top right corner de counter resetting
![terminal Screenshot](recursos/imgs/ter2.png)

in this photo you can see that the comand went thru on the bottom left corner 

![terminal Screenshot](recursos/imgs/ter1.png)

in this photo you can see the comand not changing the outcome of the counter do to the fact the value given is "false" 

![terminal Screenshot](recursos/imgs/ter3.png)


## Results

Here is a screenshot of the terminal output showing the publisher and listener in action:
![Diagram results](recursos/imgs/ss2.png)
---