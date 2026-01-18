# Chapter 6: Robot Structure and URDF

## Concept Overview

URDF (Unified Robot Description Format) is an XML-based format used to describe robot structure, including physical properties like links, joints, inertial properties, visual meshes, and collision geometries. URDF provides the bridge between abstract robot models and the physical reality of how robots are constructed and behave. In ROS 2 systems, URDF descriptions enable simulation, visualization, and control algorithms to understand the robot's physical configuration.

![URDF Structure](/img/ros2-diagrams/urdf-diagram.svg)

URDF descriptions are essential for tasks like inverse kinematics, collision detection, and robot simulation, making them a crucial component of the robotic nervous system.

## Mental Model

Think of URDF as the robot's genetic blueprint that defines its physical structure:

- **Links** are like bones - the rigid parts of the robot's skeleton
- **Joints** are like joints in the skeleton - connections that allow movement
- **Materials** are like skin and tissue - the visual properties
- **Inertial Properties** are like mass and center of gravity - how the robot responds to forces

Just as DNA encodes the structure of biological organisms, URDF encodes the structure of robotic systems, allowing software to understand and interact with the robot's physical form.

## System Architecture

### URDF Components
- **Links**: Rigid bodies with physical properties (mass, inertia, visual/collision geometry)
- **Joints**: Connections between links with specific degrees of freedom
- **Transmissions**: Mappings between software commands and hardware actuators
- **Materials**: Visual appearance properties
- **Gazebo Extensions**: Simulation-specific properties

### Integration with ROS 2
- **Robot State Publisher**: Transforms URDF into TF transforms for spatial relationships
- **RViz**: Uses URDF for visualization
- **MoveIt**: Uses URDF for motion planning and collision checking
- **Controllers**: Use URDF for joint control and feedback

### Information Flow
```
URDF Description -> Robot State Publisher -> TF Tree -> Other ROS 2 Nodes
```

## Minimal Example

Here's a simple URDF description for a basic wheeled robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/> <!-- Rotate to align with robot -->
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints connecting wheels to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.2 0.2 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.2 -0.2 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Gazebo-specific elements for simulation -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>
```

## Common Failure Modes

1. **Invalid XML Syntax**: Malformed URDF causing parser failures
2. **Missing Joint Limits**: Unlimited joints causing unexpected behavior
3. **Incorrect Mass Properties**: Wrong inertial values causing simulation instability
4. **Transform Chain Issues**: Broken kinematic chains preventing proper TF tree generation
5. **Collision Mesh Problems**: Poor collision geometry causing incorrect collision detection

## Industry Reality

In commercial robotics, URDF is used extensively for:

- **Simulation**: Creating digital twins of robots for testing and training
- **Motion Planning**: Enabling planners like MoveIt to understand robot kinematics
- **Visualization**: Providing models for tools like RViz
- **Control**: Giving controllers information about robot dynamics
- **Manufacturing**: Serving as a reference for robot assembly and maintenance

Companies often maintain multiple URDF variants for different purposes: simplified models for real-time planning, detailed models for accurate simulation, and approximate models for fast visualization.

Key URDF Components:
- Links: Rigid body elements
- Joints: Connections allowing movement
- Materials: Visual properties
- Inertial properties: Physical behavior characteristics