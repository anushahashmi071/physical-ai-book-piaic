# Chapter 2: ROS 2 Overview

## Concept Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Unlike traditional operating systems, ROS 2 provides a collection of tools, libraries, and conventions that simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. It builds upon the lessons learned from ROS 1 while addressing its limitations, particularly in the areas of real-time performance, security, and commercial deployment.

![ROS 2 Node Communication](/img/ros2-diagrams/node-topic-diagram.svg)

ROS 2 implements a distributed computing architecture that enables multiple processes (and potentially multiple machines) to coordinate to achieve complex robotic behaviors. This architecture is built around the concept of nodes that communicate through various messaging patterns.

## Mental Model

Envision ROS 2 as a sophisticated nervous system for robots where:

- **Nodes** function like specialized organs, each responsible for specific functions (sensory processing, motion planning, etc.)
- **Topics** operate like continuous information highways, constantly flowing data between components
- **Services** act like direct telephone calls, where one component requests specific information or action from another
- **Actions** resemble complex missions with progress reporting, like sending a robot on a long journey with regular status updates

This biological metaphor helps understand how ROS 2 coordinates complex behaviors through distributed components that specialize in their roles while communicating effectively with the broader system.

## System Architecture

ROS 2 uses a peer-to-peer network architecture based on the Data Distribution Service (DDS) standard. The core architectural elements include:

### Nodes
Independent processes that perform computation. Nodes are the fundamental unit of execution in ROS 2, encapsulating algorithms, sensors, actuators, or any other functional element.

### Topics and Publish-Subscribe
The primary communication mechanism where nodes publish data to named topics and other nodes subscribe to receive that data. This pattern enables asynchronous, decoupled communication.

### Services
Synchronous request-response communication pattern. A client sends a request to a service server and waits for a response.

### Actions
Goal-oriented communication pattern with feedback and cancellation capabilities. Perfect for long-running tasks that need to report progress.

### Parameters
Configuration system that allows nodes to be configured dynamically.

### Launch System
Mechanism for starting multiple nodes together with appropriate configurations.

## Minimal Example

Here's a conceptual example showing the basic ROS 2 communication patterns:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class CoreNode(Node):
    def __init__(self):
        super().__init__('core_node')

        # Topic publisher (continuous communication)
        self.publisher = self.create_publisher(String, 'system_status', 10)

        # Service server (request-response communication)
        self.service = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        # Timer to periodically publish status
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = 'System operational'
        self.publisher.publish(msg)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    core_node = CoreNode()

    try:
        rclpy.spin(core_node)
    except KeyboardInterrupt:
        pass
    finally:
        core_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Failure Modes

1. **Discovery Failures**: Nodes fail to find each other on the network due to network configuration issues
2. **QoS Incompatibilities**: Publishers and subscribers with mismatched Quality of Service profiles fail to communicate
3. **Resource Exhaustion**: High-frequency publishers can overwhelm subscribers or consume excessive bandwidth
4. **Initialization Race Conditions**: Nodes starting in wrong order causing communication failures
5. **Security Misconfigurations**: Inadequate security settings exposing the system to unauthorized access

## Industry Reality

ROS 2 has become the de facto standard for robotic development in both academic and commercial settings. Major companies like Amazon (warehouse robotics), Toyota (human support robots), and numerous autonomous vehicle manufacturers use ROS 2 for their robotic systems.

The framework's architecture addresses real-world needs:
- Commercial viability with licensing that allows proprietary extensions
- Real-time performance capabilities for time-critical applications
- Security features for deployment in sensitive environments
- Improved cross-platform support for diverse hardware platforms
- Deterministic behavior suitable for safety-critical applications

ROS 2's design philosophy emphasizes building robust, maintainable robotic systems through modular, well-defined interfaces between components.
