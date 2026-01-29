# ROS 2 Publisher and Subscriber Project

## Description

This project demonstrates a basic ROS 2 publisher–subscriber architecture.
A fixed number is published periodically on a topic, received by another node,
accumulated, and then republished. The project also includes a simple string
listener example.

---

## Nodes Overview

- **number_publisher**: Publishes a fixed integer value to `/number`
- **number_counter**: Subscribes to `/number`, accumulates the value, and publishes to `/number_count`
- **listener**: Subscribes to a string topic named `speaker`

---

## Source Code

### number_publisher.py

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int64, '/number', 10)
        self.timer_ = self.create_timer(1.0, self.publish_number)
        self.number_ = 7

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### number_counter.py

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounter(Node):

    def __init__(self):
        super().__init__('number_counter')
        self.counter_ = 0

        self.subscription_ = self.create_subscription(
            Int64,
            '/number',
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Int64,
            '/number_count',
            10
        )

    def callback(self, msg):
        self.counter_ += msg.data

        out_msg = Int64()
        out_msg.data = self.counter_
        self.publisher_.publish(out_msg)

        self.get_logger().info(
            f'Received: {msg.data} | Counter: {self.counter_}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

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

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")


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
ros2 run my_package number_publisher
ros2 run my_package number_counter
ros2 topic echo /number_count
```

---

## Terminal Output

```text
[INFO] [number_publisher]: Publishing: 7
[INFO] [number_counter]: Received: 7 | Counter: 7
[INFO] [number_publisher]: Publishing: 7
[INFO] [number_counter]: Received: 7 | Counter: 14
[INFO] [number_publisher]: Publishing: 7
[INFO] [number_counter]: Received: 7 | Counter: 21
```

---

## Conclusion

This project validates the use of ROS 2 nodes for inter-process communication
using topics. The publisher–subscriber pattern allows data to be shared and
processed in real time, serving as a foundational concept for distributed
robotic systems and future mechatronics applications.
