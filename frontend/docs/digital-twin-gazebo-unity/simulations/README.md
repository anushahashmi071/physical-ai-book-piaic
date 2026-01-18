# Digital Twin Simulation Assets

This directory contains simulation assets for the Digital Twin (Gazebo & Unity) module, including Gazebo environments, Unity scenes, and ROS 2 configurations.

## Gazebo Environments

The `gazebo-environments/` directory contains:

- `humanoid_world.world`: A complete Gazebo world file with a simple humanoid robot model and basic environment
- `simple_humanoid.urdf`: A URDF description of the humanoid robot with sensors (camera, IMU, LiDAR)

### Key Features:
- Physics configuration with realistic parameters
- Humanoid robot with torso, head, and legs
- Integrated sensors: camera, IMU, and LiDAR
- Basic obstacles for testing

## Unity Scenes

The `unity-scenes/` directory contains:

- `humanoid_scene.unity`: A Unity scene file with a basic humanoid robot visualization

### Key Features:
- Basic humanoid model with torso, head, and limbs
- Camera setup for visualization
- Lighting configuration

## ROS 2 Configurations

The `ros2-configs/` directory contains:

- `launch_humanoid_simulation.py`: A ROS 2 launch file to start the complete simulation environment

### To run the simulation:

1. Launch Gazebo with the humanoid world:
```bash
ros2 launch launch_humanoid_simulation.py
```

2. The launch file will:
   - Start Gazebo with the humanoid world
   - Spawn the robot model
   - Start robot state publisher
   - Initialize joint state publisher

## Integration

The simulation assets demonstrate the complete digital twin workflow:
1. Physics simulation in Gazebo
2. Sensor data generation
3. ROS 2 communication
4. Potential Unity visualization

This setup allows for testing of the complete simulation-first workflow described in the module documentation.