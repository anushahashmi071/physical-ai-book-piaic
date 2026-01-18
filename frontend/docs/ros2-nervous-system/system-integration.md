# Chapter 7: System Integration

## Concept Overview

System integration in ROS 2 brings together all the individual components—nodes, topics, services, actions, and URDF models—to create a cohesive robotic system. This integration encompasses the complete pipeline from sensing through perception, planning, and action, forming the unified nervous system of the robot. The integration process involves coordinating distributed components to achieve complex behaviors that emerge from the interaction of specialized functional units.

![System Integration](/img/ros2-diagrams/end-to-end-system-diagram.svg)

Effective system integration requires understanding how all ROS 2 concepts work together to create a functional robotic system.

## Mental Model

Think of system integration as orchestrating a symphony orchestra:

- **Sensors** are like instruments producing raw sounds (data)
- **Processing nodes** are like musicians interpreting the music (algorithms)
- **Communication patterns** are like the musical score coordinating everyone (ROS 2 messaging)
- **Control systems** are like conductors ensuring harmony (coordination)
- **AI agents** are like composers creating the overall composition (intelligent behavior)
- **URDF** is like the instrument specifications (robot structure)

Each component plays its role, but the magic happens when they work together in harmony to create complex, intelligent behavior.

## System Architecture

### Integration Layers
- **Hardware Abstraction**: Drivers and interfaces for sensors/actuators
- **Sensing Layer**: Data acquisition and preprocessing
- **Perception Layer**: Interpretation of sensor data
- **Planning Layer**: Decision making and trajectory generation
- **Control Layer**: Low-level actuator commands
- **Monitoring Layer**: System health and diagnostics

### Information Flow
```
Sensors -> Perception -> Planning -> Control -> Actuators
  ^                                         |
  |-----------------------------------------+
```

### Coordination Mechanisms
- **Launch Files**: Starting multiple nodes together
- **Parameters**: Configuring system-wide settings
- **TF Trees**: Maintaining spatial relationships
- **Actions**: Coordinating complex behaviors
- **Services**: Handling discrete coordination tasks

## Minimal Example

Here's an example of how different ROS 2 components integrate to form a complete system:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from example_interfaces.action import NavigateToPose

class IntegratedSystemNode(Node):
    def __init__(self):
        super().__init__('integrated_system')

        # Sensing Layer
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Perception Layer
        self.obstacle_pub = self.create_publisher(Float32, '/obstacle_distance', 10)

        # Planning/Control Layer
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Monitoring Layer
        self.status_pub = self.create_publisher(String, '/system_status', 10)

        # Internal state
        self.current_pose = None
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        # Integration timer
        self.integration_timer = self.create_timer(0.05, self.integrate_behavior)

    def scan_callback(self, msg):
        """Sensing: Process laser scan data"""
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r > 0.01 and r < 10.0], default=float('inf'))
            self.obstacle_distance = min_range
            self.obstacle_detected = min_range < 0.8  # 80cm threshold

            # Publish processed perception data
            dist_msg = Float32()
            dist_msg.data = self.obstacle_distance
            self.obstacle_pub.publish(dist_msg)

    def odom_callback(self, msg):
        """Sensing: Update pose information"""
        self.current_pose = msg.pose.pose

    def integrate_behavior(self):
        """System Integration: Coordinate all layers"""
        if self.current_pose is None:
            return

        # Planning and Control Logic
        cmd = Twist()
        status_msg = String()

        if self.obstacle_detected:
            # Reactive obstacle avoidance
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn away from obstacle
            status_msg.data = f'OBSTACLE_AVOIDANCE: Distance={self.obstacle_distance:.2f}m'
        else:
            # Continue toward goal or patrol
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            status_msg.data = f'NORMAL_OPERATION: Safe distance={self.obstacle_distance:.2f}m'

        # Execute control command
        self.cmd_vel_pub.publish(cmd)

        # Publish system status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    system = IntegratedSystemNode()

    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    finally:
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Failure Modes

1. **Timing Issues**: Different system components running at incompatible rates
2. **Resource Contention**: Multiple nodes competing for computational resources
3. **Integration Coupling**: Nodes becoming too dependent on each other's internal implementation
4. **Message Congestion**: Too many nodes publishing to shared topics
5. **Startup Sequencing**: Nodes requiring other nodes to be ready before starting

## Industry Reality

In commercial robotic systems, integration involves:

- **Modular Design**: Carefully designed interfaces between system components
- **Testing Strategies**: Extensive integration testing at multiple levels
- **Deployment Pipelines**: Automated deployment of integrated systems
- **Monitoring**: Comprehensive system health monitoring and diagnostics
- **Safety Systems**: Redundant safety layers and emergency procedures
- **Logging**: Detailed system behavior logging for debugging and improvement

Successful commercial robots typically involve years of iterative integration refinement, with careful attention to reliability, performance, and maintainability.
