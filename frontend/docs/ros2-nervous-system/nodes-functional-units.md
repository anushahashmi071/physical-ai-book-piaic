# Chapter 3: Nodes as Functional Units

## Concept Overview

Nodes are the fundamental building blocks of any ROS 2 system. A node is an independent process that performs computation, representing a single functional unit within the larger robotic system. Each node typically handles specific aspects of robot behavior, such as sensor processing, motion planning, or actuator control. This modular design enables complex systems to be built from simple, focused components that communicate through well-defined interfaces.

![Node Communication Pattern](/img/ros2-diagrams/node-mapping-diagram.svg)

In the nervous system metaphor, nodes correspond to specialized neurons or groups of neurons that perform specific functions, such as sensory processing, motor control, or higher-level coordination.

## Mental Model

Consider nodes as specialized workers in a factory, each with a specific job:

- **Sensor Nodes**: Like sensory organs, they gather information from the environment
- **Processing Nodes**: Like the brain regions, they interpret information and make decisions
- **Control Nodes**: Like motor centers, they send commands to actuators
- **Coordination Nodes**: Like executive functions, they orchestrate complex behaviors

Just as the brain operates as a network of specialized regions working together, a ROS 2 system operates as a network of specialized nodes collaborating through message passing.

## System Architecture

Nodes in ROS 2 have the following characteristics:

### Lifecycle
- **Creation**: Nodes initialize resources and establish communication interfaces
- **Execution**: Nodes perform their primary function, processing callbacks and executing timers
- **Destruction**: Nodes clean up resources and shut down gracefully

### Communication Interfaces
- **Publishers**: Send messages to topics
- **Subscribers**: Receive messages from topics
- **Service Servers**: Respond to service requests
- **Service Clients**: Make service requests
- **Action Servers**: Handle action goals with feedback
- **Action Clients**: Send action goals and receive feedback

### Resource Management
- **Timers**: Execute callbacks at regular intervals
- **Callbacks**: Handle incoming messages asynchronously
- **Parameters**: Configure node behavior dynamically

## Minimal Example

Here's an example of a node that represents a functional unit for motor control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Subscriber to receive motor commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'motor_commands',
            self.command_callback,
            10)

        # Publisher to report joint states
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)

        # Internal state tracking
        self.current_positions = [0.0, 0.0, 0.0]  # Example joint positions

        # Timer for periodic state updates
        self.timer = self.create_timer(0.05, self.update_state)  # 20 Hz

    def command_callback(self, msg):
        """Process incoming motor commands"""
        if len(msg.data) == len(self.current_positions):
            self.current_positions = list(msg.data)
            self.get_logger().info(f'Motor commands received: {self.current_positions}')
        else:
            self.get_logger().warn('Command dimension mismatch')

    def update_state(self):
        """Periodically publish joint state updates"""
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = self.current_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()

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

1. **Resource Leaks**: Nodes that don't properly clean up publishers, subscribers, or timers
2. **Callback Deadlocks**: Blocking operations in callbacks preventing other callbacks from executing
3. **Initialization Failures**: Nodes failing to properly configure their communication interfaces
4. **State Inconsistencies**: Nodes maintaining inconsistent internal state due to race conditions
5. **Graceful Shutdown Issues**: Nodes not properly cleaning up resources when terminated

## Industry Reality

In commercial robotics, nodes are often designed with specific architectural patterns:

- **Component-based Design**: Nodes may be implemented as reusable components that can be loaded dynamically
- **Lifecycle Management**: Advanced nodes implement state machines for sophisticated initialization and shutdown procedures
- **Monitoring Integration**: Production nodes often include extensive logging, health monitoring, and diagnostic capabilities
- **Fault Tolerance**: Critical nodes implement redundancy and failover mechanisms

Companies building robotic systems typically organize their software architecture around nodes that correspond to specific functional requirements, enabling teams to work on different components independently while maintaining clear interfaces between them.
