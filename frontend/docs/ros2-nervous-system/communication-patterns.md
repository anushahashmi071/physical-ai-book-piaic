# Chapter 4: Communication Patterns

## Concept Overview

ROS 2 offers three primary communication patterns that enable nodes to exchange information: Topics (publish-subscribe), Services (request-response), and Actions (goal-oriented with feedback). Each pattern serves specific use cases and has distinct characteristics that make it suitable for different types of robotic communications. Understanding when to use each pattern is crucial for designing effective robotic systems.

![Communication Patterns](/img/ros2-diagrams/concept-diagram.svg)

## Mental Model

Think of these communication patterns as different ways people communicate:

- **Topics** are like a radio broadcast - one person speaks, many listen, and the conversation is continuous
- **Services** are like a phone call - someone asks a question, waits for a specific answer
- **Actions** are like assigning a task - someone gives a goal, receives updates on progress, and can cancel if needed

This variety allows ROS 2 systems to handle everything from continuous sensor data streams to complex, long-running tasks requiring feedback.

## System Architecture

### Topics (Publish-Subscribe)
- **Pattern**: One-to-many, asynchronous communication
- **Use Case**: Continuous data streams (sensors, status updates)
- **Characteristics**:
  - Unidirectional flow from publisher to subscriber
  - No acknowledgment or guarantee of receipt
  - Data loss possible if subscribers are slow
  - Quality of Service (QoS) settings control reliability

### Services (Request-Response)
- **Pattern**: One-to-one, synchronous communication
- **Use Case**: Computation requests, configuration changes, simple queries
- **Characteristics**:
  - Request sent, response received, then communication ends
  - Synchronous - caller waits for response
  - Reliable delivery with error handling
  - Good for discrete operations

### Actions (Goal-Oriented)
- **Pattern**: One-to-one, asynchronous with feedback
- **Use Case**: Long-running tasks (navigation, manipulation)
- **Characteristics**:
  - Goal sent, feedback received during execution, result at completion
  - Asynchronous - caller doesn't wait for completion
  - Cancellation capability
  - Progress reporting during execution

## Minimal Example

Here's a comparison of the three communication patterns:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci

class CommunicationPatternsNode(Node):
    def __init__(self):
        super().__init__('communication_patterns')

        # Topic publisher and subscriber
        self.publisher = self.create_publisher(String, 'topic_messages', 10)
        self.subscriber = self.create_subscription(
            String, 'topic_messages', self.topic_callback, 10)

        # Service client and server
        self.service_client = self.create_client(AddTwoInts, 'add_two_ints')
        self.service_server = self.create_service(
            AddTwoInts, 'add_two_ints', self.service_callback)

        # Action client
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci_action')

        # Timer to demonstrate different patterns
        self.timer = self.create_timer(2.0, self.demo_patterns)

    def topic_callback(self, msg):
        """Handle incoming topic messages"""
        self.get_logger().info(f'Topic received: {msg.data}')

    def service_callback(self, request, response):
        """Handle service requests"""
        response.sum = request.a + request.b
        self.get_logger().info(f'Service: {request.a} + {request.b} = {response.sum}')
        return response

    def demo_patterns(self):
        """Demonstrate different communication patterns"""
        # Topic: Publish continuous status
        msg = String()
        msg.data = f'Demo at {self.get_clock().now().seconds_nanoseconds()}'
        self.publisher.publish(msg)

        # Service: Request computation (if service is available)
        if self.service_client.wait_for_service(timeout_sec=1.0):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            future = self.service_client.call_async(request)
            future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """Handle service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Service result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationPatternsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Failure Modes

### Topics
1. **Latching Issues**: Late-joining subscribers missing initial data
2. **QoS Mismatches**: Publishers/subscribers with incompatible reliability settings
3. **Bandwidth Saturation**: Too-frequent publications overwhelming network
4. **Message Loss**: High-frequency topics causing buffer overflows

### Services
1. **Timeout Failures**: Server not responding within expected time
2. **Connection Failures**: Service server unavailable when client calls
3. **Blocking Issues**: Service handlers taking too long, blocking other operations
4. **Serialization Errors**: Complex message types causing encoding problems

### Actions
1. **Goal Timeout**: Action server not completing goals in expected time
2. **Feedback Overload**: Too-frequent feedback overwhelming the client
3. **Cancellation Issues**: Problems with graceful goal cancellation
4. **State Synchronization**: Action server/client state mismatches

## Industry Reality

In commercial robotic systems, communication patterns are chosen based on specific requirements:

- **Topics** dominate for sensor data distribution, status monitoring, and real-time control signals
- **Services** handle configuration changes, calibration routines, and computational tasks
- **Actions** manage complex behaviors like navigation, manipulation, and coordinated multi-step tasks

Companies often establish patterns for their specific use cases, such as using latched topics for static configuration data, reliable services for safety-critical operations, and actions for mission-critical autonomous tasks.

| Pattern | Type | Use Case | Characteristics |
|---------|------|----------|-----------------|
| Topics | Async Pub/Sub | Continuous data | Unidirectional, no guarantees |
| Services | Sync Request/Response | Discrete operations | Synchronous, reliable |
| Actions | Async Goal-Oriented | Long-running tasks | Feedback, cancellation |