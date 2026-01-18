# Data Model: Digital Twin (Gazebo & Unity)

**Date**: 2026-01-18
**Feature**: 002-digital-twin-gazebo-unity

## Key Entities

### Digital Twin
**Definition**: Virtual replica of a physical humanoid robot used for simulation and testing
**Attributes**:
- robot_model: URDF/SDF description of the physical robot
- simulation_state: Current pose, velocities, and sensor readings
- synchronization_status: Degree of alignment with physical robot state
- fidelity_level: Accuracy rating of the digital representation

### Gazebo Environment
**Definition**: Physics-based simulation environment for robotic systems
**Attributes**:
- world_description: SDF file defining the environment
- physics_engine: Selected physics engine (ODE, Bullet, DART)
- gravity_settings: Gravitational constants applied to environment
- real_time_factor: Simulation speed relative to real time

### Unity Scene
**Definition**: Visualization environment for 3D robotic models and interactions
**Attributes**:
- scene_graph: Hierarchical structure of 3D objects
- lighting_config: Environmental lighting and shadows
- material_properties: Visual appearance of objects
- rendering_pipeline: Forward/deferred rendering configuration

### Humanoid Robot Model
**Definition**: 3D model of a bipedal robot with articulated joints and sensors
**Attributes**:
- joint_configurations: Degrees of freedom and limits for each joint
- link_properties: Mass, inertia, and collision properties
- sensor_mounts: Positions and types of attached sensors
- actuator_specs: Motor specifications and control parameters

### Sensor Simulation
**Definition**: Virtual representations of physical sensors (cameras, LiDAR, IMUs) in the simulation
**Attributes**:
- sensor_type: Camera, LiDAR, IMU, etc.
- noise_parameters: Statistical models for sensor noise
- field_of_view: Angular range of sensor coverage
- update_rate: Frequency of sensor data publication

### Physics Engine
**Definition**: System that calculates realistic movement, collisions, and forces in the simulation
**Attributes**:
- solver_type: Method used to solve physics equations
- time_step: Discrete time interval for physics calculations
- damping_factors: Energy dissipation parameters
- contact_models: Algorithms for collision response

### ROS 2 Integration
**Definition**: Connection between simulation environment and ROS 2 communication framework
**Attributes**:
- topic_mappings: Correspondence between Gazebo topics and ROS 2 topics
- message_types: ROS 2 message types used for communication
- tf_frames: Coordinate frame relationships
- qos_profiles: Quality of Service settings for communication

### Simulation Workflow
**Definition**: Process of designing, testing, and validating robotic systems in virtual environments
**Attributes**:
- simulation_phases: Development, testing, validation stages
- validation_metrics: Criteria for comparing simulation to reality
- iteration_cycle: Process for refining simulation accuracy
- deployment_strategy: Transition from simulation to physical implementation

## Entity Relationships

```
[Digital Twin] --(contains)--> [Humanoid Robot Model]
[Digital Twin] --(uses)--> [Gazebo Environment]
[Digital Twin] --(uses)--> [Unity Scene]
[Humanoid Robot Model] --(has)--> [Sensor Simulation]
[Humanoid Robot Model] --(has)--> [Physics Engine]
[Gazebo Environment] --(integrates with)--> [ROS 2 Integration]
[Unity Scene] --(represents)--> [Gazebo Environment]
[Sensor Simulation] --(connected via)--> [ROS 2 Integration]
[Physics Engine] --(configured in)--> [Gazebo Environment]
```

## State Transitions

### Digital Twin States
- **Initial**: Model created but not yet validated
- **Calibrated**: Parameters adjusted to match physical robot
- **Operational**: Running simulation with valid results
- **Degraded**: Some aspects of twin deviating from physical system
- **Synchronized**: Real-time alignment with physical robot state

### Simulation Environment States
- **Setup**: Environment configured but not running
- **Initializing**: Physics engine and entities being loaded
- **Running**: Active simulation with real-time or accelerated time
- **Paused**: Simulation temporarily suspended
- **Stopped**: Simulation terminated

## Data Flow Patterns

### Sensor Data Flow
Digital Twin → Gazebo Environment → Sensor Simulation → ROS 2 Integration → Processing Nodes

### Control Command Flow
ROS 2 Integration → Gazebo Environment → Humanoid Robot Model → Actuator Commands

### Visualization Flow
Gazebo Environment → Unity Scene → Rendering Pipeline → User Interface