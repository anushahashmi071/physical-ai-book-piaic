# Data Model: ROS 2 as Robotic Nervous System

**Date**: 2026-01-17
**Feature**: 001-ros2-nervous-system

## Key Entities

### Node
**Definition**: Independent functional unit in the ROS 2 distributed system that performs specific robot functions
**Attributes**:
- name: string identifier
- namespace: hierarchical grouping
- lifecycle: active/inactive state
**Relationships**: Publishes/subscribes to Topics; provides/calls Services; sends/cancels Actions

### Topic
**Definition**: Continuous data stream mechanism for publishing and subscribing to sensor data and other ongoing information
**Attributes**:
- name: string identifier
- message_type: standardized format
- qos_profile: quality of service settings
**Relationships**: Nodes publish to and subscribe from Topics

### Service
**Definition**: Request-response interaction pattern for discrete operations that have immediate results
**Attributes**:
- name: string identifier
- request_type: input message format
- response_type: output message format
**Relationships**: Nodes provide Services and call Services

### Action
**Definition**: Long-running, goal-oriented task mechanism with feedback and cancellation capabilities
**Attributes**:
- name: string identifier
- goal_type: goal message format
- result_type: result message format
- feedback_type: feedback message format
**Relationships**: Nodes send Action goals and cancel Actions

### Message
**Definition**: Data packet that carries information between ROS 2 components using standardized formats
**Attributes**:
- timestamp: when message was created
- content: actual data payload
- header: metadata for routing
**Relationships**: Messages flow through Topics, Services, and Actions

### rclpy
**Definition**: Python client library that enables Python-based AI agents to interface with ROS 2
**Attributes**:
- version: library version
- node_interface: connection to ROS 2 network
- executor: event loop management
**Relationships**: Connects Python AI Agents to ROS 2 Nodes, Topics, Services, Actions

### URDF (Unified Robot Description Format)
**Definition**: XML-based format that describes the physical structure and properties of humanoid robots
**Attributes**:
- links: rigid bodies with physical properties
- joints: connections between links
- materials: visual appearance
- transmissions: motor interfaces
**Relationships**: Describes Robot Controllers and their connections

### Python AI Agent
**Definition**: Software component written in Python that implements artificial intelligence algorithms for robot control
**Attributes**:
- behavior_logic: AI algorithms
- sensor_processing: perception functions
- action_selection: decision making
**Relationships**: Connects to ROS 2 via rclpy; communicates with Robot Controllers

### Robot Controller
**Definition**: Component responsible for translating high-level commands into low-level motor controls
**Attributes**:
- control_loop: frequency and timing
- motor_interfaces: connections to hardware
- safety_limits: operational boundaries
**Relationships**: Receives commands from Python AI Agents; controls robot hardware

## Entity Relationships

```
[Python AI Agent] --(connects via)--> [rclpy] --(interfaces with)--> [Node]
[Node] --(publishes/subscribes to)--> [Topic] --(carries)--> [Message]
[Node] --(provides/calls)--> [Service] --(exchanges)--> [Message]
[Node] --(initiates/cancels)--> [Action] --(manages)--> [Message]
[URDF] --(describes)--> [Robot Controller]
[Python AI Agent] --(communicates with)--> [Robot Controller] --(controls)--> [Robot Hardware]
```

## Data Flow Patterns

### Sensor Data Flow
Python AI Agent ← rclpy ← Node ← Topic ← Sensor Messages

### Control Command Flow
Python AI Agent → rclpy → Node → Topic/Service/Action → Robot Controller

### Feedback Loop
Robot Controller → Topic → Node → rclpy → Python AI Agent