# Research: Digital Twin (Gazebo & Unity)

**Date**: 2026-01-18
**Feature**: 002-digital-twin-gazebo-unity

## Digital Twin Fundamentals

### Definition and Role in Robotics
Digital twins are virtual replicas of physical systems that enable real-time monitoring, simulation, and optimization. In robotics, digital twins serve as:
- Testing environments for algorithms before physical deployment
- Prototyping platforms for complex robot behaviors
- Training grounds for AI agents in safe, reproducible conditions
- Validation tools for multi-robot coordination strategies

### Simulation vs Real-World Systems
Key differences between simulation and real-world systems:
- **Physics Approximation**: Simulations use simplified physics models vs. complex real-world interactions
- **Sensory Noise**: Real sensors have noise, drift, and failure modes not perfectly modeled in simulation
- **Latency**: Communication delays and processing times differ between simulated and real systems
- **Environmental Factors**: Weather, lighting, and dynamic obstacles vary in reality vs. controlled simulation

## Physics Simulation Basics

### Rigid Bodies, Gravity, Collisions
- **Rigid Body Dynamics**: Objects maintain constant shape during simulation, simplifying calculations
- **Gravity Modeling**: Constant downward acceleration (9.81 m/s²) applied to all objects with mass
- **Collision Detection**: Algorithms to determine when objects intersect and respond appropriately
- **Contact Resolution**: Methods to resolve forces when objects collide (penetration depth, restitution)

### Time Steps and Stability
- **Fixed Time Steps**: Simulation advances in discrete time increments for predictable behavior
- **Stability Requirements**: Small time steps needed for numerical stability in physics calculations
- **Real-Time Factor**: Ratio of simulation speed to real-time (RTF=1.0 for real-time simulation)

## Gazebo for Robotics Simulation

### Environment and Robot Setup
- **SDF/URDF Models**: XML formats describing robot structure, kinematics, and dynamics
- **World Files**: XML descriptions of simulation environments with static and dynamic objects
- **Plugin Architecture**: Gazebo uses plugins for sensors, controllers, and custom behaviors
- **ROS 2 Integration**: gazebo_ros_pkgs package provides ROS 2 interfaces for Gazebo

### Running Humanoid Simulations
- **Joint Control**: Position, velocity, or effort control for articulated robot joints
- **Sensor Simulation**: Cameras, LiDAR, IMUs, force/torque sensors in virtual environment
- **Ground Truth**: Access to perfect state information not available in real systems
- **Simulation Speed**: Adjustable real-time factor to run faster or slower than real-time

## Sensor Simulation

### Cameras, LiDAR, IMU
- **Camera Simulation**: RGB, depth, and stereo vision with adjustable parameters
- **LiDAR Simulation**: 2D and 3D laser scanning with configurable resolution and range
- **IMU Simulation**: Accelerometer and gyroscope data with realistic noise models

### Sensor Noise and Limitations
- **Noise Models**: Gaussian, uniform, and bias noise added to simulate real sensor imperfections
- **Limited Field of View**: Cameras and LiDAR have restricted sensing ranges
- **Occlusion Handling**: Objects hidden behind others are not sensed
- **Dynamic Range**: Sensors have limits on detectable signal strengths

## Unity for Visualization

### Scene Setup and Humanoid Interaction
- **3D Scene Composition**: GameObjects, components, and hierarchy organization
- **Humanoid Rigging**: Avatar setup with humanoid bone mapping for animation
- **Material and Lighting**: Visual appearance and environmental effects
- **Interaction Systems**: Input handling and user interface elements

### High-Level Visualization Concepts
- **Rendering Pipeline**: Forward vs. deferred rendering for performance optimization
- **Level of Detail (LOD)**: Multiple representations of objects for performance
- **Occlusion Culling**: Techniques to avoid rendering invisible objects

## ROS 2 Integration

### Connecting Simulated Sensors to ROS 2 Nodes
- **Bridge Architecture**: gazebo_ros_pkgs provides ROS 2 publishers/subscribers for Gazebo
- **Topic Mapping**: Sensor data published to standard ROS 2 message types
- **TF Frames**: Coordinate transformations between robot parts and world
- **Action/Service Integration**: Higher-level behaviors using ROS 2 actions and services

### Data Flow Between Sim and Control
- **Sensor Data Flow**: Simulated sensors → ROS 2 topics → Processing nodes
- **Control Command Flow**: Control nodes → ROS 2 topics → Actuator commands in simulation
- **Parameter Server**: Dynamic configuration of simulation parameters via ROS 2

## Simulation-First Workflow

### Debugging in Simulation
- **Reproducibility**: Identical initial conditions allow consistent debugging
- **Visualization Tools**: Enhanced debugging with ground truth and overlay information
- **Failure Injection**: Controlled testing of failure scenarios without physical risk
- **Performance Profiling**: Easier identification of computational bottlenecks

### Limits of Digital Twins
- **Reality Gap**: Differences between simulated and real physics/modeling
- **Emergent Behaviors**: Real-world behaviors may not appear in simulation
- **Hardware Specifics**: Motor dynamics, sensor characteristics may differ from models
- **Environmental Complexity**: Real environments more complex than simulated ones

## Technical Decisions

### Gazebo Version Choice
Selected Gazebo Garden or Fortress for:
- Better ROS 2 integration
- Improved performance and stability
- Active development and community support
- Plugin architecture enhancements

### Unity Version Choice
Selected Unity 2022.3 LTS for:
- Long-term support and stability
- ROS 2 integration packages availability
- Cross-platform deployment capabilities
- Extended support for 3D visualization needs

### ROS 2 Distribution
Selected ROS 2 Humble Hawksbill for:
- Long-term support (until May 2027)
- Mature Gazebo integration
- Extensive documentation and community support
- Industrial adoption and stability